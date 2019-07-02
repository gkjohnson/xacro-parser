# xacro-parser


<!-- [![npm version](https://img.shields.io/npm/v/xacro-parser.svg?style=flat-square)](https://www.npmjs.com/package/xacro-parser) -->
[![travis build](https://img.shields.io/travis/gkjohnson/xacro-parser.svg?style=flat-square)](https://travis-ci.org/gkjohnson/xacro-parser)
[![lgtm code quality](https://img.shields.io/lgtm/grade/javascript/g/gkjohnson/xacro-parser.svg?style=flat-square&label=code-quality)](https://lgtm.com/projects/g/gkjohnson/xacro-parser/)

Javascript parser and loader for processing the [ROS Xacro file format](http://wiki.ros.org/xacro). 

**NOTE**
_This package uses [new Function](https://github.com/gkjohnson/xacro-parser/blob/master/src/XacroParser.js#L146), which can be unsafe to evaluate. While an effort has been made to sanitize the expressions in the xacro file it is not complete._

_Docs in progress and do not reflect current API_

# Use

## Loading Files From Disk

```js
import fs from 'fs';
import { XacroParser } from 'xacro-parser';

xacroParser.getFileContents = path => {

  return fs.readFile(path, { encoding: 'utf8' });

};

xacroParser.load('./path/to/file.xacro').then(xml => {

  // ... xml file ...

});
```

## Loading Files from Server

```js

import fs from 'fs';
import { XacroParser } from 'xacro-parser';

xacroParser.getFileContents = path => {

  return fetch(path).then(res => res.text());

};

xacroParser.load('./path/to/file.xacro').then(xml => {

  // ... xml file ...

});
```

# API

## XacroParser

### .localProperties

### .requirePrefix

### .inOrder

### .workingPath

### .rospackCommands

### .parse

### .load

