# Unit Test App

ESP-IDF unit tests are run using Unit Test App. The app can be built with the unit tests for a specific component. Unit tests are in `test` subdirectories of respective components.

# Building Unit Test App

## GNU Make

* Follow the setup instructions in the top-level esp-idf README.
* Set IDF_PATH environment variable to point to the path to the esp-idf top-level directory.
* `make menuconfig` to configure the Unit Test App. Under Component Config select "Run unit tests instead of user_main."
* Run `make -j $(grep -c ^processor /proc/cpuinfo) TEST_COMPONENTS=` with `TEST_COMPONENTS` set to names of the components to be included in the test app. E.g. `make TEST_COMPONENTS="user_main cJSON"` **DO NOT USE `make TESTS_ALL=1`.**
  * You can use `make -j $(grep -c ^processor /proc/cpuinfo) TEST_COMPONENTS=` to increase compilation speed.
* Follow the printed instructions to flash, or run `make flash`.

# Flash Size

The unit test partition table assumes a 1MB flash size. Building all tests is not supported. The partition table has been edited to work for 1MB which restricts the number of tests supported.

# Running Unit Tests

The unit test loader will prompt by showing a menu of available tests to run:

* Type a number to run a single test.
* `*` to run all tests.
* `[tagname]` to run tests with "tag"
* `![tagname]` to run tests without "tag"
* `"test name here"` to run test with given name

# Making Components

To make a new component, copy the user_main, and rename it to the component name. You should rename the .c and .h files as well.

# Adding .c and .h files

To add a:
| File you wish to add | Directory to place it in |               Example             |
|:--------------------:|:------------------------:|:---------------------------------:|
| `.c`                 | `components/[component]` | `components/[component]/[file.c]` |
| `.h`                 | `components/[component]` | `components/[component]/[file.c]` |
| `.c` (for testing)   | `components/[component]` | `components/[component]/[file.c]` |
| `.h` (for testing)   | `components/[component]` | `components/[component]/[file.c]` |
