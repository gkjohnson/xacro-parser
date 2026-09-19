<!-- This file is generated automatically. Do not edit it directly. -->
# xacro-parser

## XacroParser

Parser for processing the [ROS Xacro file format](http://wiki.ros.org/xacro). Xacro files from
different versions of ROS require different options to be set. The differences are documented
in the [spec](http://wiki.ros.org/xacro).

Options required for xacros created with a ROS version <= release 8 (ROS Indigo):

```js
parser.inOrder = false;
parser.requirePrefix = false;
parser.localProperties = false;
```

Options required for xacros created with a ROS version >= release 9 (ROS Jade):

```js
parser.inOrder = true;
parser.requirePrefix = true;
parser.localProperties = true;
```

> [!NOTE]
> XacroParser depends on the browser xml parser. When running in Node a `DOMParser`
> implementation such as the one provided by `jsdom` must be assigned to `global.DOMParser`.

### .inOrder

```js
inOrder: boolean = true
```

Since `ROS Jade` xacro allows for [in order](http://wiki.ros.org/xacro#Processing_Order)
processing, which allows variables to be used to define include paths and order-dependent
property definitions. The equivalent of the `--inorder` xacro command line flag.


### .requirePrefix

```js
requirePrefix: boolean = true
```

Since `ROS Jade` xacro [requires all tags be prefixed with "xacro:"](http://wiki.ros.org/xacro#Deprecated_Syntax).
Setting `requirePrefix` to false disables this requirement.


### .localProperties

```js
localProperties: boolean = true
```

Since `ROS Jade` xacro [scopes property definitions to the containing macro](http://wiki.ros.org/xacro#Local_properties).
Setting `localProperties` to false disables this behavior.


### .rospackCommands

```js
rospackCommands: Object<string, function(...string): string> | ( command: string, args: string ) => string = {}
```

A map of rospack command stem to handling function that take all arguments as function
parameters. An example implementation of the `rospack find` command:

```js
parser.rospackCommands =
  {

    find: function( pkg ) {

      switch( pkg ) {

        case 'valkyrie_description':
          return '/absolute/path/to/valkyrie_description/';
        case 'r2_description':
          return '/absolute/path/to/r2_description/'

      }

    }

  };
```

Alternatively a function can be provided to evaluate the command:

```js
parser.rospackCommands = ( command, ...args ) => {

    if ( command === 'find' ) {

        const [ pkg ] = args;
        switch( pkg ) {
            case 'valkyrie_description':
                return '/absolute/path/to/valkyrie_description/';
            case 'r2_description':
                return '/absolute/path/to/r2_description/'
        }

    }

};
```


### .arguments

```js
arguments: Object<string, (string|number|boolean)> = {}
```

A map of argument names to values that will be substituted for `$(arg name)` tags.

```js
parser.arguments =
  {
    transmission_hw_interface: "hardware_interface/PositionJointInterface",
    arm_x_separation: -0.4,
    laser_visual: true,
  };
```

> [!NOTE]
> These take precedence over any `<xacro:arg>` defaults.

### .workingPath

```js
workingPath: string = ''
```

The working directory to search for dependent files in when parsing `include` tags.

> [!NOTE]
> The path is required to end with '/'.

### .getFileContents

```js
async getFileContents( path: string ): Promise<string>
```

An overrideable function that takes a file path and returns the contents of that file as a
string. Used for loading a documents referenced in `include` tags.


### .parse

```js
async parse( data: string ): Promise<XMLDocument>
```

Parses the passed xacro contents using the options specified on the object and returns an
xml document of the processed xacro file.


## XacroLoader

_extends [`XacroParser`](#xacroparser)_

Extends XacroParser and implements `getFileContents` to load from a server using fetch.

```js
const loader = new XacroLoader();
loader.load(
    '../path/to/file.xacro',
    result => {

        // xacro XML

    },
    err => {

        // parse error

    } );
```

> [!NOTE]
> The working path is extracted automatically from the loaded url. Only works in the browser.

### .fetchOptions

```js
fetchOptions: RequestInit = {}
```

Options passed to `fetch` when loading files.


### .load

```js
load(
	url: string,
	onComplete: ( result: XMLDocument ) => void,
	onError: ( error: Error ) => void
): void
```

Loads and parses the xacro file at the given url. If `workingPath` has not been set it is
extracted from the url.


### .parse

```js
parse(
	data: string,
	onComplete: ( result: XMLDocument ) => void,
	onError: ( error: Error ) => void
): void
```

Parses the passed xacro contents using the options specified on the object and calls
`onComplete` with the xml document of the processed xacro file.


### .getFileContents

```js
getFileContents( path: string ): Promise<string>
```

Loads the file at the given path using `fetch` and `fetchOptions` and returns the contents
as a string.

