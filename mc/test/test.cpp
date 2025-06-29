/*
 Copyright (c) 2014-present PlatformIO <contact@platformio.org>

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
**/


#include <unity.h>

void setUp(void) {
    // set stuff up here
}

void tearDown(void) {
    // clean stuff up here
}

//----------------------------------------------------------------------------
bool is_valid_pin_for_esp32_super_mini_MIKE(const unsigned char pin)
//----------------------------------------------------------------------------    
{
    return (0x3003FFUL>>pin) & 1;
}

void mike5(void) {
    TEST_ASSERT_EQUAL(32, is_valid_pin_for_esp32_super_mini_MIKE(5));
}

void test_expensive_operation(void) {
    TEST_IGNORE();
}

void RUN_UNITY_TESTS() {
    UNITY_BEGIN();
    RUN_TEST(mike5);
    RUN_TEST(test_expensive_operation);
    UNITY_END();
}

void app_main(void)
{
    RUN_UNITY_TESTS();
}


