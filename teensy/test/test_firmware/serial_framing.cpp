/**
 * SERIAL MONITOR TESTS -- SerialMonitorBase, USBSerialMonitor, RadioSerialMonitor.
 * Tests in this file cover the byte-level state machine that turns a stream of serial bytes into a validated packet.
 */
#include "test_support.h"
#include "suites.hpp"
#include "Monitors/USBSerialMonitor.hpp"
#include "Monitors/RadioSerialMonitor.hpp"


// Assumptions the framing depends on -- if one of these ever fails, the other tests in this file are meaningless.
static void test_framing_flags_are_distinct() {
    TEST_ASSERT_NOT_EQUAL_MESSAGE(constants::serial::RX_START_FLAG, constants::serial::RX_END_FLAG,
                                  "Start and end flags must differ or packets cannot be delimited");
}

static void test_buffers_are_non_empty() {
    TEST_ASSERT_GREATER_THAN_size_t(0, constants::serial::USB_BUFFER_LEN);
    TEST_ASSERT_GREATER_THAN_size_t(0, constants::serial::RADIO_BUFFER_LEN);
}


// Tests that validate everything went well.
static void test_usb_valid_packet_is_accepted() {
    const std::vector<uint8_t> payload = make_payload(constants::serial::USB_BUFFER_LEN);
    feed(Serial, frame_packet(payload));

    USBSerialMonitor monitor;
    monitor.execute();

    TEST_ASSERT_TRUE_MESSAGE(sfr::serial::update_servos_usb, "A well-formed USB packet should raise the update flag");
    TEST_ASSERT_EQUAL_UINT8_ARRAY(payload.data(), sfr::serial::usb_buffer, constants::serial::USB_BUFFER_LEN);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(0, sfr::serial::dropped_packets, "A valid packet must not count as dropped");
}

static void test_radio_valid_packet_is_accepted() {
    const std::vector<uint8_t> payload = make_payload(constants::serial::RADIO_BUFFER_LEN);
    feed(Serial2, frame_packet(payload));

    RadioSerialMonitor monitor;
    monitor.execute();

    TEST_ASSERT_TRUE_MESSAGE(sfr::serial::update_servos_radio, "A well-formed radio packet should raise the flag");
    TEST_ASSERT_EQUAL_UINT8_ARRAY(payload.data(), sfr::serial::radio_buffer, constants::serial::RADIO_BUFFER_LEN);
    TEST_ASSERT_EQUAL_UINT8(0, sfr::serial::dropped_packets);
}

static void test_radio_publishes_mode_flag_from_payload() {
    std::vector<uint8_t> payload = make_payload(constants::serial::RADIO_BUFFER_LEN);
    payload[layout::RADIO_FLAG] = 0;
    feed(Serial2, frame_packet(payload));

    RadioSerialMonitor monitor;
    monitor.execute();

    TEST_ASSERT_EQUAL_UINT8_MESSAGE(0, sfr::serial::radio_flag,
                                    "radio_flag must mirror payload byte 0 so the boat can switch out of radio mode");
}

static void test_packet_split_across_execute_calls_still_completes() {
    const std::vector<uint8_t> payload = make_payload(constants::serial::USB_BUFFER_LEN);
    const std::vector<uint8_t> packet = frame_packet(payload);
    const size_t split = packet.size() / 2;

    USBSerialMonitor monitor;

    // First half arrives; the packet is still mid-flight so nothing should be published yet.
    Serial.mock_rx(packet.data(), split);
    monitor.execute();
    TEST_ASSERT_FALSE_MESSAGE(sfr::serial::update_servos_usb, "A half-received packet must not be published");

    // Remainder arrives on a later loop iteration.
    Serial.mock_rx(packet.data() + split, packet.size() - split);
    monitor.execute();

    TEST_ASSERT_TRUE_MESSAGE(sfr::serial::update_servos_usb, "The packet should complete once the rest arrives");
    TEST_ASSERT_EQUAL_UINT8_ARRAY(payload.data(), sfr::serial::usb_buffer, constants::serial::USB_BUFFER_LEN);
    TEST_ASSERT_EQUAL_UINT8(0, sfr::serial::dropped_packets);
}

static void test_back_to_back_packets_both_parse() {
    const std::vector<uint8_t> first = make_payload(constants::serial::USB_BUFFER_LEN);
    std::vector<uint8_t> second = make_payload(constants::serial::USB_BUFFER_LEN);
    second[0] = static_cast<uint8_t>(second[0] + 1); // Make the second packet distinguishable.
    if (second[0] == constants::serial::RX_START_FLAG || second[0] == constants::serial::RX_END_FLAG) second[0] += 1;

    feed(Serial, frame_packet(first));
    feed(Serial, frame_packet(second));

    USBSerialMonitor monitor;
    monitor.execute();

    TEST_ASSERT_TRUE(sfr::serial::update_servos_usb);
    TEST_ASSERT_EQUAL_UINT8(0, sfr::serial::dropped_packets);
    TEST_ASSERT_EQUAL_UINT8_ARRAY_MESSAGE(second.data(), sfr::serial::usb_buffer, constants::serial::USB_BUFFER_LEN,
                                          "The most recent packet should win when several arrive in one pass");
}


// Tests that validate what happens with malformed packets.
static void test_short_packet_is_dropped() {
    const std::vector<uint8_t> payload = make_payload(constants::serial::USB_BUFFER_LEN - 1);
    feed(Serial, frame_packet(payload));

    USBSerialMonitor monitor;
    monitor.execute();

    TEST_ASSERT_FALSE_MESSAGE(sfr::serial::update_servos_usb, "A truncated packet must never be published");
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(1, sfr::serial::dropped_packets, "Truncated packets should be counted as dropped");
}

static void test_overlong_packet_is_dropped() {
    const std::vector<uint8_t> payload = make_payload(constants::serial::USB_BUFFER_LEN + 1);
    feed(Serial, frame_packet(payload));

    USBSerialMonitor monitor;
    monitor.execute();

    TEST_ASSERT_FALSE_MESSAGE(sfr::serial::update_servos_usb, "An over-long packet must never be published");
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(1, sfr::serial::dropped_packets, "Over-long packets should be counted as dropped");
}

static void test_radio_short_packet_is_dropped() {
    const std::vector<uint8_t> payload = make_payload(constants::serial::RADIO_BUFFER_LEN - 1);
    feed(Serial2, frame_packet(payload));

    RadioSerialMonitor monitor;
    monitor.execute();

    TEST_ASSERT_FALSE(sfr::serial::update_servos_radio);
    TEST_ASSERT_EQUAL_UINT8(1, sfr::serial::dropped_packets);
}

static void test_dropped_packet_counter_accumulates() {
    USBSerialMonitor monitor;

    feed(Serial, frame_packet(make_payload(constants::serial::USB_BUFFER_LEN - 1)));
    monitor.execute();
    feed(Serial, frame_packet(make_payload(constants::serial::USB_BUFFER_LEN + 1)));
    monitor.execute();

    TEST_ASSERT_EQUAL_UINT8_MESSAGE(2, sfr::serial::dropped_packets, "Each rejected packet bumps the drop counter");
}

static void test_stray_bytes_outside_a_packet_are_ignored() {
    Serial.mock_rx({safe_payload_byte(0), safe_payload_byte(1), constants::serial::RX_END_FLAG});

    USBSerialMonitor monitor;
    monitor.execute();

    TEST_ASSERT_FALSE(sfr::serial::update_servos_usb);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(0, sfr::serial::dropped_packets, "Noise outside a packet is not a dropped packet");
}

static void test_start_flag_mid_packet_restarts_cleanly() {
    const std::vector<uint8_t> payload = make_payload(constants::serial::USB_BUFFER_LEN);

    std::vector<uint8_t> stream;
    stream.push_back(constants::serial::RX_START_FLAG);
    stream.push_back(safe_payload_byte(200)); // Represents a partial payload that gets abandoned.
    const std::vector<uint8_t> restarted = frame_packet(payload);
    stream.insert(stream.end(), restarted.begin(), restarted.end());
    feed(Serial, stream);

    USBSerialMonitor monitor;
    monitor.execute();

    TEST_ASSERT_TRUE_MESSAGE(sfr::serial::update_servos_usb, "The restarted packet should parse normally");
    TEST_ASSERT_EQUAL_UINT8_ARRAY(payload.data(), sfr::serial::usb_buffer, constants::serial::USB_BUFFER_LEN);
}


// Tests that validate stale packet timeouts.
static void test_stalled_packet_times_out_and_is_dropped() {
    USBSerialMonitor monitor;

    // A packet starts arriving but stops partway through.
    Serial.mock_rx({constants::serial::RX_START_FLAG, safe_payload_byte(0)});
    monitor.execute();
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(0, sfr::serial::dropped_packets, "Nothing is stale yet");

    // Time passes with no further bytes, then the rest finally shows up far too late.
    mock_advance_millis(constants::serial::RX_PACKET_TIMEOUT_MS + 1);
    const std::vector<uint8_t> payload = make_payload(constants::serial::USB_BUFFER_LEN);
    Serial.mock_rx(payload.data() + 1, payload.size() - 1);
    Serial.mock_rx_byte(constants::serial::RX_END_FLAG);
    monitor.execute();

    TEST_ASSERT_EQUAL_UINT8_MESSAGE(1, sfr::serial::dropped_packets, "The stalled packet should be dropped");
    TEST_ASSERT_FALSE_MESSAGE(sfr::serial::update_servos_usb, "Late leftovers must not be assembled into a packet");
}

static void test_packet_just_inside_timeout_still_completes() {
    USBSerialMonitor monitor;
    const std::vector<uint8_t> payload = make_payload(constants::serial::USB_BUFFER_LEN);

    Serial.mock_rx_byte(constants::serial::RX_START_FLAG);
    monitor.execute();

    mock_advance_millis(constants::serial::RX_PACKET_TIMEOUT_MS - 1);
    Serial.mock_rx(payload.data(), payload.size());
    Serial.mock_rx_byte(constants::serial::RX_END_FLAG);
    monitor.execute();

    TEST_ASSERT_TRUE_MESSAGE(sfr::serial::update_servos_usb, "A packet inside the timeout window should complete");
    TEST_ASSERT_EQUAL_UINT8(0, sfr::serial::dropped_packets);
}


// Tests that validate port isolation (different monitors must not consume each other's traffic).
static void test_usb_monitor_ignores_radio_port() {
    feed(Serial2, frame_packet(make_payload(constants::serial::RADIO_BUFFER_LEN)));

    USBSerialMonitor monitor;
    monitor.execute();

    TEST_ASSERT_FALSE_MESSAGE(sfr::serial::update_servos_usb, "The USB monitor must not read the radio UART");
    TEST_ASSERT_GREATER_THAN_size_t_MESSAGE(0, Serial2.mock_rx_pending(),
                                            "Radio bytes should still be waiting for the radio monitor");
}

static void test_radio_monitor_ignores_usb_port() {
    feed(Serial, frame_packet(make_payload(constants::serial::USB_BUFFER_LEN)));

    RadioSerialMonitor monitor;
    monitor.execute();

    TEST_ASSERT_FALSE_MESSAGE(sfr::serial::update_servos_radio, "The radio monitor must not read the USB UART");
    TEST_ASSERT_GREATER_THAN_size_t(0, Serial.mock_rx_pending());
}


// Runner.
void run_serial_framing_tests() {
    Unity.TestFile = __FILE__; // Report failures against this file, not main.cpp.
    RUN_TEST(test_framing_flags_are_distinct);
    RUN_TEST(test_buffers_are_non_empty);

    RUN_TEST(test_usb_valid_packet_is_accepted);
    RUN_TEST(test_radio_valid_packet_is_accepted);
    RUN_TEST(test_radio_publishes_mode_flag_from_payload);
    RUN_TEST(test_packet_split_across_execute_calls_still_completes);
    RUN_TEST(test_back_to_back_packets_both_parse);

    RUN_TEST(test_short_packet_is_dropped);
    RUN_TEST(test_overlong_packet_is_dropped);
    RUN_TEST(test_radio_short_packet_is_dropped);
    RUN_TEST(test_dropped_packet_counter_accumulates);
    RUN_TEST(test_stray_bytes_outside_a_packet_are_ignored);
    RUN_TEST(test_start_flag_mid_packet_restarts_cleanly);

    RUN_TEST(test_stalled_packet_times_out_and_is_dropped);
    RUN_TEST(test_packet_just_inside_timeout_still_completes);

    RUN_TEST(test_usb_monitor_ignores_radio_port);
    RUN_TEST(test_radio_monitor_ignores_usb_port);
}
