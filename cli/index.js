const fs = require('fs');
const path = require('path');
const { XacroParser } = require('../src/XacroParser.js');
const { JSDOM } = require('jsdom');

const jsdom = new JSDOM();
const window = jsdom.window;
global.DOMParser = window.DOMParser;
global.XMLSerializer = window.XMLSerializer;

const args = process.argv;
const inputFile = args[2];
const inOrder = !args.includes('--oldorder');

if (args.includes('--help')) {

    console.log('xacro-parser <input-file> [--oldorder]');
    process.exit(0);

}

const parser = new XacroParser();
parser.inOrder = inOrder;
parser.requirePrefix = inOrder;
parser.localProperties = inOrder;
parser.workingPath = path.dirname(inputFile);
parser.getFileContents = p => {

    return fs.readFileSync(p, { encoding: 'utf8' });

};

parser
    .parse(parser.getFileContents(inputFile))
    .then(xml => {

        const serializer = new XMLSerializer();
        console.log(serializer.serializeToString(xml));

    });
