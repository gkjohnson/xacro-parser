# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/en/1.0.0/)
and this project adheres to [Semantic Versioning](http://semver.org/spec/v2.0.0.html).

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
