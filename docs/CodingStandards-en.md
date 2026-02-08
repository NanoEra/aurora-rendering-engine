# NanoEra Project Design and Coding Standards V1.0
## 1. Modules
1. A module is the fundamental unit of a project. Each module should have one and only one well-defined functionality.
2. A single module should consist of the following two parts:
    * Module Declaration: Contains the module's header files (`.h` or `.hpp`), used to declare the module's interfaces, data structures, etc.
    * Module Implementation: Contains the module's source code files (`.cpp` or `.c`), used to implement the module's functional logic.
* Note: For more complex individual modules, multiple header files and source code files may be used, but they must all be organized within a folder named after the module.
3. Dependencies between modules are allowed. However, circular dependencies in module declarations must be杜绝 to ensure module independence and maintainability. *Specifically, circular dependencies in module implementations are permitted. For example, it is acceptable for `B.cpp` to include `A.h` and for `A.cpp` to include `B.h`. However, any referencing relationship between `A.h` and `B.h` is prohibited.*
4. The reserved folder names listed in `extended_folders.list` in **Section 2.4** should not be used as module names.

## 2. File and Directory Organization Standards
1. The project root directory should contain at least the following folders:
    * `include/`: Stores header files.
    * `src/`: Stores source code files.
    * `lib/`: Stores third-party library files.
    * `build/`: Stores files generated during compilation. ***(This folder should be created automatically by the build program during project construction and deleted automatically during clean builds, not created manually.)***
    * `res/`: Stores resource files, such as project icons, etc.
    * `docs/`: Stores project documentation.
    * `tests/`: Stores test code.
    * `scripts/`: Stores build and deployment scripts.
    * `experiments/`: Used for archiving project test records and temporary experimental code and data.
* In addition to these, other folders can be created in the project root directory. However, the purpose of any such folder must be documented during `git push`.
2. Special requirements for the header file folder (`include/`):
    * In principle, modules within the project's `include/` folder that do not depend on any non-compiler-built-in libraries and external modules should be stored in the `basic/` and `external/` subfolders, respectively.
    * Additionally, folders can be created for module categorization. Since some modules are stored in folders rather than single files, an `extended_folders.list` file can be created within the `include/` folder to declare folders used for module categorization, with one folder name per line (formatted as in this document). Accordingly, these reserved folder names should not be used as module names.
3. Special requirements for the source code folder (`src/`):
    * The structure of source code files should mirror that of their corresponding header files.
4. The `scripts/` folder may contain the following script files:
    * `export_modules_requirements.sh`: Exports project module dependencies based on the source code in `include/` and `src/`, and `include/extended_folders.list`.
    * `check_code.sh`: Used for checking code standards compliance.
5. All files should uniformly use UTF-8 encoding. If multiple encodings are necessary, corresponding conversion scripts can be created within the `scripts/` folder.

## 3. Coding Standards
1. Naming Conventions:
    * Folder and file names should use lowercase words separated by underscores.
    * Function and variable names should use lowercase words separated by underscores.
    * Class, struct, and other data structure names should use PascalCase. Member variables and functions of data structures should be suffixed with an underscore.
    * Global constants, macro definitions, and macro functions should use uppercase words separated by underscores.
    * Global variables should be prefixed with `g_` and use lowercase words separated by underscores.
2. Requirements for header file inclusion during coding:
    * Header files in the `include/` folder should only include headers strictly necessary for declaring module interfaces and data structures. Other headers, required only during implementation, should be included in the implementation files.
    * Implementation files in the `src/` folder should likewise only include headers strictly necessary for that specific file. They must not include headers that are already included in the module's header file (since the module's implementation file will always include the module's declaration header, this avoids conflicting includes).
