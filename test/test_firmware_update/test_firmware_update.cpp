// Firmware update over CAN: the module's receiver against a stand-in flash and a
// stand-in dash, including lost frames, lost answers and a damaged image.
#include <cassert>
#include <cstdio>
#include <vector>
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

int main() {
  test_checksums_match_the_published_ones();
  test_a_whole_image_arrives_through_lost_frames_and_lost_answers();
  test_a_wrong_image_is_refused_and_the_running_firmware_is_kept();
  test_nothing_happens_by_accident();
  test_a_transfer_the_dash_walks_away_from_is_given_up();
  test_new_firmware_is_on_trial_until_the_dash_confirms_the_right_build();
  std::puts("Firmware update receiver tests passed");
}
