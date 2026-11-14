// Firmware update over CAN: the module's receiver against a stand-in flash and a
// stand-in dash, including lost frames, lost answers and a damaged image.
#include <cassert>
#include <cstdio>
#include <vector>
#include <zlib.h>
#include <can_contract/firmware_update.h>

namespace fw = can_protocol::firmware;
using can_protocol::CanFrame;

struct Flash {
  std::vector<uint8_t> image;
  uint32_t capacity = 0x330000;
  bool open = false, failWrite = false, failFinish = false, finished = false, confirmed = false;
  int begins = 0, aborts = 0;
  bool begin(uint32_t size) { ++begins; if (size > capacity) return false; image.clear(); open = true; return true; }
  bool write(const uint8_t* data, uint32_t length) {
    if (!open || failWrite) return false;
    image.insert(image.end(), data, data + length); return true;
  }
  bool finish() { open = false; finished = !failFinish; return finished; }
  void abort() { ++aborts; open = false; image.clear(); }
  bool confirm() { confirmed = true; return true; }
};

struct Bench {
  Flash flash;
  fw::Receiver<Flash> module{fw::TARGET_GATEWAY, 0x1A2B3C4D, flash};
  uint32_t now = 1000;
  CanFrame reply{};
  fw::Action action = fw::Action::NONE;
  bool send(const CanFrame& frame) { ++now; return module.handle(frame, reply, now, action); }
  uint8_t status() const { return reply.data[3]; }
  uint16_t value() const { return can_protocol::decodeU16BE(reply.data[4], reply.data[5]); }
};

static std::vector<uint8_t> makeImage(uint32_t size) {
  std::vector<uint8_t> image(size);
  uint32_t seed = 12345;
  for (auto& byte : image) { seed = seed * 1664525u + 1013904223u; byte = static_cast<uint8_t>(seed >> 24); }
  return image;
}

// Sends one block as the dash does. `drop` skips a frame; `damage` flips a bit in one.
static bool sendBlock(Bench& bench, const std::vector<uint8_t>& image, uint16_t block, int drop = -1, int damage = -1) {
  const uint32_t offset = block * fw::BLOCK_BYTES;
  const uint32_t length = image.size() - offset < fw::BLOCK_BYTES ? image.size() - offset : fw::BLOCK_BYTES;
  const uint8_t frames = static_cast<uint8_t>((length + 6) / 7);
  for (uint8_t i = 0; i < frames; ++i) {
    if (i == drop) continue;
    const uint32_t at = offset + i * 7u;
    const uint8_t count = static_cast<uint8_t>(length - i * 7u < 7 ? length - i * 7u : 7);
    CanFrame frame = fw::packData(i, image.data() + at, count);
    if (i == damage) frame.data[3] ^= 0x10;
    assert(!bench.send(frame));                       // Data frames are never answered.
  }
  return bench.send(fw::packBlockEnd(fw::TARGET_GATEWAY, block, fw::crc16(image.data() + offset, length), frames));
}

static void test_checksums_match_the_published_ones() {
  const uint8_t text[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  assert(fw::crc16(text, 9) == 0x29B1);               // CRC-16/CCITT-FALSE check value
  assert(fw::crc32(text, 9) == 0xCBF43926UL);         // zlib check value
  assert(fw::crc32(text + 4, 5, fw::crc32(text, 4)) == 0xCBF43926UL);   // Continued in pieces.
}

static void test_a_whole_image_arrives_through_lost_frames_and_lost_answers() {
  Bench bench;
  const auto image = makeImage(10000);                // 22 full blocks and a part block of 144 bytes
  assert(bench.send(fw::packQuery(fw::TARGET_GATEWAY)));
  assert(bench.reply.id == fw::ID_REPLY && bench.reply.data[0] == fw::INFO && fw::getU32(bench.reply.data + 2) == 0x1A2B3C4D);
  assert(bench.reply.data[6] == 0 && bench.reply.data[7] == fw::VERSION);
  assert(bench.send(fw::packBegin(fw::TARGET_GATEWAY, image.size())) && bench.status() == fw::STATUS_OK);
  assert(bench.action == fw::Action::STARTED && bench.module.active());
  bench.send(fw::packQuery(fw::TARGET_GATEWAY));
  assert(bench.reply.data[6] == fw::FLAG_UPDATING);
  const uint16_t blocks = static_cast<uint16_t>((image.size() + fw::BLOCK_BYTES - 1) / fw::BLOCK_BYTES);
  for (uint16_t block = 0; block < blocks; ++block) {
    if (block == 3) {                                 // A frame lost on the bus.
      assert(sendBlock(bench, image, block, 17) && bench.status() == fw::STATUS_RESEND && bench.value() == block);
    }
    if (block == 5) {                                 // A frame damaged: caught by the block's CRC.
      assert(sendBlock(bench, image, block, -1, 40) && bench.status() == fw::STATUS_RESEND);
    }
    assert(sendBlock(bench, image, block) && bench.status() == fw::STATUS_OK && bench.value() == block);
    if (block == 8) {                                 // The answer was lost, so the dash sends the block again:
      const size_t written = bench.flash.image.size();
      assert(sendBlock(bench, image, block) && bench.status() == fw::STATUS_OK && bench.value() == block);
      assert(bench.flash.image.size() == written);    // acknowledged, not written twice.
    }
  }
  assert(bench.module.received() == image.size());
  assert(bench.send(fw::packEnd(fw::TARGET_GATEWAY, fw::crc32(image.data(), image.size()))));
  assert(bench.status() == fw::STATUS_OK && bench.action == fw::Action::RESTART);
  assert(bench.flash.finished && bench.flash.image == image && !bench.module.active());
}

static void test_a_wrong_image_is_refused_and_the_running_firmware_is_kept() {
  const auto image = makeImage(900);
  {
    Bench bench;                                      // Whole-image CRC does not match.
    bench.send(fw::packBegin(fw::TARGET_GATEWAY, image.size()));
    for (uint16_t block = 0; block < 3; ++block) assert(sendBlock(bench, image, block) && bench.status() == fw::STATUS_OK);
    assert(bench.send(fw::packEnd(fw::TARGET_GATEWAY, 0xDEADBEEF)) && bench.status() == fw::STATUS_CHECK_FAILED);
    assert(bench.action == fw::Action::STOPPED && !bench.flash.finished && bench.flash.aborts == 1 && !bench.module.active());
  }
  {
    Bench bench;                                      // END before everything arrived.
    bench.send(fw::packBegin(fw::TARGET_GATEWAY, image.size()));
    sendBlock(bench, image, 0);
    assert(bench.send(fw::packEnd(fw::TARGET_GATEWAY, fw::crc32(image.data(), image.size()))) && bench.status() == fw::STATUS_CHECK_FAILED);
    assert(!bench.flash.finished);
  }
  {
    Bench bench;                                      // The image's own checksum fails in the flash layer.
    bench.flash.failFinish = true;
    bench.send(fw::packBegin(fw::TARGET_GATEWAY, image.size()));
    for (uint16_t block = 0; block < 3; ++block) sendBlock(bench, image, block);
    assert(bench.send(fw::packEnd(fw::TARGET_GATEWAY, fw::crc32(image.data(), image.size()))) && bench.status() == fw::STATUS_CHECK_FAILED);
    assert(bench.action == fw::Action::STOPPED);
  }
  {
    Bench bench;                                      // Too large for the slot, and an empty one.
    bench.flash.capacity = 500;
    assert(bench.send(fw::packBegin(fw::TARGET_GATEWAY, 501)) && bench.status() == fw::STATUS_TOO_LARGE && !bench.module.active());
    assert(bench.send(fw::packBegin(fw::TARGET_GATEWAY, 0)) && bench.status() == fw::STATUS_TOO_LARGE);
  }
  {
    Bench bench;                                      // Flash refuses a write.
    bench.send(fw::packBegin(fw::TARGET_GATEWAY, image.size()));
    bench.flash.failWrite = true;
    assert(sendBlock(bench, image, 0) && bench.status() == fw::STATUS_FLASH_ERROR && !bench.module.active());
  }
}

static void test_nothing_happens_by_accident() {
  Bench bench;
  const auto image = makeImage(448);
  // Addressed to another module, the wrong length, or without the two key bytes: ignored.
  assert(!bench.send(fw::packQuery(fw::TARGET_TAILLIGHT)));
  assert(!bench.send(fw::packBegin(fw::TARGET_TAILLIGHT, 448)));
  CanFrame plain = fw::packBegin(fw::TARGET_GATEWAY, 448);
  plain.data[5] = 0;
  assert(!bench.send(plain) && !bench.module.active() && bench.flash.begins == 0);
  CanFrame shorter = fw::packBegin(fw::TARGET_GATEWAY, 448);
  shorter.dlc = 7;
  assert(!bench.send(shorter));
  CanFrame other = fw::packBegin(fw::TARGET_GATEWAY, 448);
  other.id = 0x502;
  assert(!bench.send(other));
  // Data and block ends with no transfer open change nothing.
  assert(!bench.send(fw::packData(0, image.data(), 7)));
  assert(bench.send(fw::packBlockEnd(fw::TARGET_GATEWAY, 0, 0, 64)) && bench.status() == fw::STATUS_BAD_STATE);
  assert(bench.send(fw::packEnd(fw::TARGET_GATEWAY, 0)) && bench.status() == fw::STATUS_BAD_STATE);
  assert(bench.flash.image.empty() && !bench.flash.finished);
  // A block out of order, or with the wrong frame count, is refused and names the one expected.
  bench.send(fw::packBegin(fw::TARGET_GATEWAY, 2000));
  assert(bench.send(fw::packBlockEnd(fw::TARGET_GATEWAY, 2, 0, 64)) && bench.status() == fw::STATUS_BAD_STATE && bench.value() == 0);
  assert(bench.send(fw::packBlockEnd(fw::TARGET_GATEWAY, 0, 0, 10)) && bench.status() == fw::STATUS_BAD_STATE);
  // The dash can call it off; a new BEGIN starts cleanly.
  assert(bench.send(fw::packAbort(fw::TARGET_GATEWAY)) && bench.status() == fw::STATUS_OK && bench.action == fw::Action::STOPPED);
  assert(!bench.module.active() && bench.flash.aborts == 1);
  assert(bench.send(fw::packAbort(fw::TARGET_GATEWAY)) && bench.action == fw::Action::NONE);
}

static void test_a_transfer_the_dash_walks_away_from_is_given_up() {
  Bench bench;
  const auto image = makeImage(2000);
  bench.send(fw::packBegin(fw::TARGET_GATEWAY, image.size()));
  sendBlock(bench, image, 0);
  assert(!bench.module.expired(bench.now + fw::IDLE_ABORT_MS - 10));
  assert(bench.module.expired(bench.now + fw::IDLE_ABORT_MS + 10));
  assert(!bench.module.active() && bench.flash.aborts == 1);
  assert(!bench.module.expired(bench.now + 60000));    // Only once.
  // Each frame keeps it alive, across the 32-bit millisecond wrap too.
  Bench late;
  late.now = 0xFFFFFF00u;
  late.send(fw::packBegin(fw::TARGET_GATEWAY, image.size()));
  late.now += 3000;                                   // Wraps past zero.
  sendBlock(late, image, 0);
  assert(!late.module.expired(late.now + 1000) && late.module.active());
}

static void test_new_firmware_is_on_trial_until_the_dash_confirms_the_right_build() {
  Bench bench;
  bench.module.setOnTrial(true);
  bench.send(fw::packQuery(fw::TARGET_GATEWAY));
  assert(bench.reply.data[6] == fw::FLAG_ON_TRIAL);
  assert(bench.send(fw::packConfirm(fw::TARGET_GATEWAY, 0x99999999)) && bench.status() == fw::STATUS_WRONG_BUILD);
  assert(bench.module.onTrial() && !bench.flash.confirmed);
  assert(bench.send(fw::packConfirm(fw::TARGET_GATEWAY, 0x1A2B3C4D)) && bench.status() == fw::STATUS_OK);
  assert(bench.action == fw::Action::CONFIRMED && !bench.module.onTrial() && bench.flash.confirmed);
  bench.flash.confirmed = false;                      // Asked again: fine, nothing to do.
  assert(bench.send(fw::packConfirm(fw::TARGET_GATEWAY, 0x1A2B3C4D)) && bench.status() == fw::STATUS_OK);
  assert(bench.action == fw::Action::NONE && !bench.flash.confirmed);
}

static void test_the_dash_can_restart_a_module_but_not_during_a_transfer() {
  Bench bench;
  assert(bench.send(fw::command(fw::RESTART, fw::TARGET_GATEWAY)) && bench.status() == fw::STATUS_OK);
  assert(bench.reply.data[2] == fw::RESTART && bench.action == fw::Action::RESTART);
  assert(!bench.send(fw::command(fw::RESTART, fw::TARGET_TAILLIGHT)));     // Another module's: ignored
  const auto image = makeImage(2000);
  bench.send(fw::packBegin(fw::TARGET_GATEWAY, image.size()));
  assert(bench.send(fw::command(fw::RESTART, fw::TARGET_GATEWAY)) && bench.status() == fw::STATUS_BUSY);
  assert(bench.action == fw::Action::NONE && bench.module.active());     // The transfer carries on
}

// ---- Version 2: 8 KB blocks, only the missing frames again, compression ----

// The ESP32 boards unpack with their ROM's decompressor; here zlib stands in for it.
struct ZlibInflate {
  static constexpr bool kAvailable = true;
  int begins = 0, ends = 0;
  bool begin() { ++begins; return true; }
  void end() { ++ends; }
  bool unpack(const uint8_t* in, uint32_t inLength, uint8_t* out, uint32_t outLength) {
    uLongf got = outLength;
    return uncompress(out, &got, in, inLength) == Z_OK && got == outLength;
  }
};

template <class Inflate>
struct Bench2 {
  Flash flash;
  fw::Receiver<Flash, Inflate> module{fw::TARGET_GATEWAY, 0x1A2B3C4D, flash};
  uint32_t now = 1000;
  CanFrame reply{};
  fw::Action action = fw::Action::NONE;
  bool send(const CanFrame& frame) { ++now; return module.handle(frame, reply, now, action); }
  uint8_t status() const { return reply.data[3]; }
  uint16_t value() const { return can_protocol::decodeU16BE(reply.data[4], reply.data[5]); }
  uint16_t last() const { return can_protocol::decodeU16BE(reply.data[6], reply.data[7]); }
};

static std::vector<uint8_t> chunkOf(const std::vector<uint8_t>& image, uint16_t block) {
  const uint32_t offset = block * fw::CHUNK_BYTES;
  const uint32_t length = image.size() - offset < fw::CHUNK_BYTES ? image.size() - offset : fw::CHUNK_BYTES;
  return std::vector<uint8_t>(image.begin() + offset, image.begin() + offset + length);
}

static std::vector<uint8_t> deflated(const std::vector<uint8_t>& bytes) {
  uLongf length = compressBound(bytes.size());
  std::vector<uint8_t> out(length);
  assert(compress2(out.data(), &length, bytes.data(), bytes.size(), 9) == Z_OK);
  out.resize(length);
  return out;
}

// Frames from..to (inclusive) of a block as sent, skipping `drop`.
template <class B>
static void sendFrames(B& bench, const std::vector<uint8_t>& wire, uint16_t from, uint16_t to, const std::vector<uint16_t>& drop = {}) {
  for (uint16_t i = from; i <= to; ++i) {
    bool skip = false;
    for (uint16_t d : drop) skip = skip || d == i;
    if (skip) continue;
    const uint32_t at = i * 6u;
    const uint8_t count = static_cast<uint8_t>(wire.size() - at < 6 ? wire.size() - at : 6);
    assert(!bench.send(fw::packData2(i, wire.data() + at, count)));
  }
}

template <class B>
static bool sendEnd2(B& bench, const std::vector<uint8_t>& wire, uint16_t block) {
  return bench.send(fw::packBlockEnd2(fw::TARGET_GATEWAY, block, fw::crc16(wire.data(), wire.size()), static_cast<uint16_t>(wire.size())));
}

static uint16_t framesOf(const std::vector<uint8_t>& wire) { return static_cast<uint16_t>((wire.size() + 5) / 6); }

// Firmware-like: runs of repeated bytes among random ones, so it compresses about as much.
static std::vector<uint8_t> makeCompressibleImage(uint32_t size) {
  std::vector<uint8_t> image = makeImage(size);
  for (uint32_t i = 0; i < size; i += 64) for (uint32_t j = i; j < i + 24 && j < size; ++j) image[j] = static_cast<uint8_t>(i >> 6);
  return image;
}

static void test_version_2_compressed_blocks_and_only_the_missing_frames_again() {
  Bench2<ZlibInflate> bench;
  const auto image = makeCompressibleImage(20000);    // Two full 8 KB blocks and one of 3616 bytes
  assert(bench.send(fw::packQuery(fw::TARGET_GATEWAY)));
  assert(bench.reply.data[7] == 2 && bench.reply.data[6] == fw::FLAG_COMPRESSION);
  assert(bench.send(fw::packBegin(fw::TARGET_GATEWAY, image.size(), fw::MODE_LARGE | fw::MODE_COMPRESSED)) && bench.status() == fw::STATUS_OK);
  assert(bench.module.active() && bench.flash.begins == 1);
  for (uint16_t block = 0; block < 3; ++block) {
    const auto wire = deflated(chunkOf(image, block));
    assert(wire.size() < chunkOf(image, block).size());
    const uint16_t frames = framesOf(wire);
    if (block == 1) {
      // Frames 5 and 40 lost: the module names the range, keeps the rest; only that range again.
      sendFrames(bench, wire, 0, frames - 1, {5, 40});
      assert(sendEnd2(bench, wire, block) && bench.status() == fw::STATUS_MISSING && bench.value() == 5 && bench.last() == 40);
      sendFrames(bench, wire, 5, 40);
    } else {
      sendFrames(bench, wire, 0, frames - 1);
    }
    assert(sendEnd2(bench, wire, block) && bench.status() == fw::STATUS_OK && bench.value() == block);
    if (block == 2) {                                 // Its answer was lost: acknowledged again, not written twice.
      const size_t written = bench.flash.image.size();
      assert(sendEnd2(bench, wire, block) && bench.status() == fw::STATUS_OK && bench.value() == block);
      assert(bench.flash.image.size() == written);
    }
  }
  assert(bench.module.received() == image.size());
  assert(bench.send(fw::packEnd(fw::TARGET_GATEWAY, fw::crc32(image.data(), image.size()))));
  assert(bench.status() == fw::STATUS_OK && bench.action == fw::Action::RESTART);
  assert(bench.flash.finished && bench.flash.image == image && !bench.module.active());
}

static void test_version_2_without_compression_and_a_damaged_frame() {
  Bench2<fw::NoInflate> bench;                       // A module that cannot unpack:
  assert(bench.send(fw::packQuery(fw::TARGET_GATEWAY)) && bench.reply.data[6] == 0);
  const auto image = makeImage(9000);
  assert(bench.send(fw::packBegin(fw::TARGET_GATEWAY, image.size(), fw::MODE_LARGE | fw::MODE_COMPRESSED)));
  assert(bench.status() == fw::STATUS_BAD_STATE && !bench.module.active());   // compressed is refused,
  assert(bench.send(fw::packBegin(fw::TARGET_GATEWAY, image.size(), fw::MODE_LARGE)) && bench.status() == fw::STATUS_OK);
  const auto first = chunkOf(image, 0);               // plain 8 KB blocks work.
  sendFrames(bench, first, 0, framesOf(first) - 1);
  CanFrame damaged = fw::packData2(100, first.data() + 600, 6);
  damaged.data[4] ^= 0x01;
  bench.send(damaged);                                // A damaged frame: the CRC catches it, the block again.
  assert(sendEnd2(bench, first, 0) && bench.status() == fw::STATUS_RESEND);
  sendFrames(bench, first, 0, framesOf(first) - 1);
  assert(sendEnd2(bench, first, 0) && bench.status() == fw::STATUS_OK);
  const auto second = chunkOf(image, 1);
  sendFrames(bench, second, 0, framesOf(second) - 1);
  assert(sendEnd2(bench, second, 1) && bench.status() == fw::STATUS_OK);
  assert(bench.send(fw::packEnd(fw::TARGET_GATEWAY, fw::crc32(image.data(), image.size()))) && bench.status() == fw::STATUS_OK);
  assert(bench.flash.image == image);
}

static void test_version_2_refuses_what_it_cannot_use() {
  const auto image = makeCompressibleImage(10000);
  {
    Bench2<ZlibInflate> bench;                        // Unknown mode bits, or compressed without large blocks.
    assert(bench.send(fw::packBegin(fw::TARGET_GATEWAY, image.size(), 0x04)) && bench.status() == fw::STATUS_BAD_STATE);
    assert(bench.send(fw::packBegin(fw::TARGET_GATEWAY, image.size(), fw::MODE_COMPRESSED)) && bench.status() == fw::STATUS_BAD_STATE);
    assert(!bench.module.active() && bench.flash.begins == 0);
  }
  {
    Bench2<ZlibInflate> bench;                        // A block that does not unpack to its share: the transfer ends.
    bench.send(fw::packBegin(fw::TARGET_GATEWAY, image.size(), fw::MODE_LARGE | fw::MODE_COMPRESSED));
    const auto wrong = deflated(chunkOf(image, 1));   // Block 1's bytes (1808), sent as block 0 (8192)
    sendFrames(bench, wrong, 0, framesOf(wrong) - 1);
    assert(sendEnd2(bench, wrong, 0) && bench.status() == fw::STATUS_CHECK_FAILED);
    assert(bench.action == fw::Action::STOPPED && !bench.module.active() && bench.flash.aborts == 1);
    assert(bench.module.received() == 0);
  }
  {
    Bench2<ZlibInflate> bench;                        // The wrong block, and a version 1 transfer still works.
    bench.send(fw::packBegin(fw::TARGET_GATEWAY, image.size(), fw::MODE_LARGE | fw::MODE_COMPRESSED));
    const auto wire = deflated(chunkOf(image, 0));
    sendFrames(bench, wire, 0, framesOf(wire) - 1);
    assert(sendEnd2(bench, wire, 3) && bench.status() == fw::STATUS_BAD_STATE && bench.value() == 0);
    assert(bench.send(fw::packBegin(fw::TARGET_GATEWAY, 448)) && bench.status() == fw::STATUS_OK);
    const auto small = makeImage(448);
    for (uint8_t i = 0; i < 64; ++i) bench.send(fw::packData(i, small.data() + i * 7, 7));
    assert(bench.send(fw::packBlockEnd(fw::TARGET_GATEWAY, 0, fw::crc16(small.data(), 448), 64)) && bench.status() == fw::STATUS_OK);
    assert(bench.send(fw::packEnd(fw::TARGET_GATEWAY, fw::crc32(small.data(), 448))) && bench.status() == fw::STATUS_OK);
    assert(bench.flash.image == small);
  }
  {
    Bench2<ZlibInflate> bench;                        // Abandoned: its buffers are given back.
    bench.send(fw::packBegin(fw::TARGET_GATEWAY, image.size(), fw::MODE_LARGE | fw::MODE_COMPRESSED));
    bench.send(fw::packAbort(fw::TARGET_GATEWAY));
    assert(!bench.module.active());
  }
}

int main() {
  test_checksums_match_the_published_ones();
  test_a_whole_image_arrives_through_lost_frames_and_lost_answers();
  test_a_wrong_image_is_refused_and_the_running_firmware_is_kept();
  test_nothing_happens_by_accident();
  test_a_transfer_the_dash_walks_away_from_is_given_up();
  test_new_firmware_is_on_trial_until_the_dash_confirms_the_right_build();
  test_the_dash_can_restart_a_module_but_not_during_a_transfer();
  test_version_2_compressed_blocks_and_only_the_missing_frames_again();
  test_version_2_without_compression_and_a_damaged_frame();
  test_version_2_refuses_what_it_cannot_use();
  std::puts("Firmware update receiver tests passed");
}
