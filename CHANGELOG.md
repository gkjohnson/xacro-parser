# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/en/1.0.0/)
and this project adheres to [Semantic Versioning](http://semver.org/spec/v2.0.0.html).

## [0.3.4] - 2021-12-09
### Added
- Support for the "=" syntax as well as ":=" for macro parameters.

## [0.3.3] - 2021-08-05
### Fixed
- Type definitions not working for Typescript users

## [0.3.2] - 2021-07-21
### Fixed
- XacroLoader typescript definitions.
- Node package.json incorrectly using a module file in the "main" field.

## [0.3.1] - 2021-04-26
### Fixed
- Type definitions not exporting correctly.

## [0.3.0] - 2021-04-23
### Added
- Support for default macro values specified using single quotes.
- Support for `not` operator.

### Changed
- Added `"type": "module"` and `"sideEffects": false` to the package.json.

### Fixed
- Typescript definitions exporting the `XacroLoader` and `XacroParser` classes incorrectly.
- `!` operator performing a factorial rather than a boolean inversion.
- `||` operator performing concatenation rather than boolean OR. Note that the `&&` operator is not currently supported by `expr-eval` which is used for expressions. Instead `and` can be used.

### Removed
- Unneeded dependency.

## [0.2.3] - 2021-01-01

### Added
- Option to pass a function as the rospack commands option.

### Fixed
- XacroLoader: throw a human readable error if fetch fails.

## [0.2.2] - 2020-12-28

### Added
- Support for `radians` and `degrees` functions in xacro expressions.

### Fix
- README typo.
- Case where a file separator would be added to the working path even if it wasn't specified.

## [0.2.1] - 2020-05-25

### Fix
- Dependency warnings.

## [0.2.0] - 2020-01-31

### Changed
- Package and build to use a single index.js file.

## [0.1.1] - 2020-01-31

### Added
- Add support for basic math functions in expressions.

### Changed
- Use `expr-eval` package instead of `new Function` for expression evaluation.
- Remove comments that preceed xacro elements.
- Throw errors rather than log warnings when a xacro fails to be created properly.

## [0.1.0] - 2020-01-09

### Changed

- Changed the XacroLoader `parse` and `load` signatures to accept an `onError` callback and removed the options parameter.

## [0.0.1] - 2019-08-19

- Initial release.
