#include <Arduino.h>
#include <unity.h>

void test_addition(void) {
    TEST_ASSERT_EQUAL(4, 2 + 2);
}

void setup() {
    delay(2000);
    UNITY_BEGIN();
    RUN_TEST(test_addition);
    UNITY_END();
}

void loop() {}