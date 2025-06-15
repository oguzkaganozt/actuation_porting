# C++ Language Features in Zephyr

Zephyr currently provides only a subset of C++ functionality.

## Unsupported Features

The following features are not supported:

* Static global object destruction
* OS-specific C++ standard library classes (e.g. `std::thread`, `std::mutex`)

## Supported Features

While not an exhaustive list, support for the following functionality is included:

* Inheritance
* Virtual functions
* Virtual tables
* Static global object constructors
* Dynamic object management with the `new` and `delete` operators
* Exceptions
* RTTI
* Standard Template Library (STL)

## Important Notes

* Do not use C++ for kernel, driver, or system initialization code.
* Static global object constructors are initialized after the drivers are initialized but before the application `main()` function. Therefore, use of C++ is restricted to application code.
* In order to make use of C++ exceptions, the `CONFIG_CPP_EXCEPTIONS` must be selected in the application configuration file.

## More info at <https://docs.zephyrproject.org/3.6.0/develop/languages/cpp/index.html>