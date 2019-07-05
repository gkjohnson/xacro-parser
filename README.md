# xacro-parser


<!-- [![npm version](https://img.shields.io/npm/v/xacro-parser.svg?style=flat-square)](https://www.npmjs.com/package/xacro-parser) -->
[![travis build](https://img.shields.io/travis/gkjohnson/xacro-parser.svg?style=flat-square)](https://travis-ci.org/gkjohnson/xacro-parser)
[![lgtm code quality](https://img.shields.io/lgtm/grade/javascript/g/gkjohnson/xacro-parser.svg?style=flat-square&label=code-quality)](https://lgtm.com/projects/g/gkjohnson/xacro-parser/)

Javascript parser and loader for processing the [ROS Xacro file format](http://wiki.ros.org/xacro). 

**NOTE**
_This package uses [new Function](https://github.com/gkjohnson/xacro-parser/blob/master/src/XacroParser.js#L146), which can be unsafe to evaluate. While an effort has been made to sanitize the expressions in the xacro file it is not guaranteed to be complete._

# Use

## Loading Files From Disk

```js
import fs from 'fs';
import { XacroParser } from 'xacro-parser';
import { JSDOM } from 'jsdom';

// XacroParser depends on the browser xml parser.
global.DOMParser = new JSDOM().window.DOMParser;

const parser = new XacroParser();
parser.workingPath = './path/to/directory/';
parser.getFileContents = path => fs.readFile(path, { encoding: 'utf8' });

const xacroContents = fs.readFileSync('./path/to/directory/file.xacro', { encoding: 'utf8' });
parser.parse(xacroContents).then(result => {

  // xacro XML

});

```

## Loading Files from Server

```js

import fs from 'fs';
import { XacroParser } from 'xacro-parser';

const parser = new XacroParser();
parser.workingPath = './path/to/directory/';
parser.getFileContents = path => fetch(path).then(res => res.text());

const xacroContents = fetch('./path/to/directory/file.xacro', { encoding: 'utf8' });
parser.parse(xacroContents).then(result => {

  // xacro XML

});

```

## Using the Loader

```js
import { XacroLoader } from 'xacro-parser';

// The working path is extracted automatically.
// Only works in the browser.
const loader = new XacroLoader();
loader.load('../path/to/file.xacro').then(result => {

  // xacro XML

});
```

# API

## XacroParser

### .localProperties

```js
localProperties = true : boolean
```

Since `ROS Jade` xacro [scopes property definitions to the containing macro](http://wiki.ros.org/xacro#Local_properties). Setting `localProperties` to false disables this behavior.

### .requirePrefix

```js
requirePrefix = true : boolean
```

Since `ROS Jade` xacro [requires all tags be prefixed with "xacro:"](http://wiki.ros.org/xacro#Deprecated_Syntax). Setting `requirePrefix` to false disables this requirement.

### .inOrder

```js
inOrder = true : boolean
```

Since `ROS Jade` xacro allows for [in order](http://wiki.ros.org/xacro#Processing_Order) processing, which allows variables to be used to define include paths and order-dependent property definitions. The equivalent of the `--inorder` xacro command line flag.

### .workingPath

```js
workingPath = '' : string
```

The working directory to search for dependent files in when parsing `include` tags. The path is required to end with '/'.

### .rospackCommands

```js
rospackCommands = {} : Object
```

A map of rospack command stem to handling function that take all arguments as function parameters. An example implementation of the `rospack find` command:

```js
{
  find: function(pkg) {
    switch(pkg) {
      case 'valkyrie_description:
        return '/absolute/path/to/valkyrie_description/';
      case 'r2_description:
        return '/absolute/path/to/r2_description/'
    }
  }
}
```

### .parse

```js
parse( contents : string ) : Promise<XMLDocument>
```

Parses the passed xacro contents using the options specified on the object and returns an xml document of the processed xacro file.

### .getFileContents

```js
getFileContents( path : string ) : Promise<string>
```

And overrideable function that takes a file path and returns the contents of that file as a string. Used for loading a documents referenced in `include` tags.

# Limitations and Missing Features

## Unimplemented Features

- Substituation args using the default arg tags and the `$(arg val)` command are not supported [#3](https://github.com/gkjohnson/xacro-parser/issues/3).
- Macro argument pass-through using `param:=^|default` is not supported [#5](https://github.com/gkjohnson/xacro-parser/issues/5).
- Calling macros with a dynamic name using the `<xacro:call macro="${var}"/>` syntax is not supported [#9](https://github.com/gkjohnson/xacro-parser/issues/9). 
- Include tag namespaces are not supported [#12](https://github.com/gkjohnson/xacro-parser/issues/12).

## Limitations

- The official xacro parser supports using basically any Python syntax in the `${}` syntax which can't be easily supported in Javascript. Instead basic argument substitution and expression evaluation is supported but this does not include yaml, arrays, dictionaries, or math functions like sin or cos [#15](https://github.com/gkjohnson/xacro-parser/issues/15), [#17](https://github.com/gkjohnson/xacro-parser/issues/17).
- Evaluation uses `new Function` to evaluate expressions, which can be unsafe. An effort has been made to sanitize the expressions but it is not guaranteed to be complete [#4](https://github.com/gkjohnson/xacro-parser/issues/4).
