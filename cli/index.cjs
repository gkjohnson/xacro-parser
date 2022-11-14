#!/usr/bin/env node
const fs = require('fs');
const path = require('path');
const { XacroParser } = require('../umd/index.cjs');
const { JSDOM } = require('jsdom');

const jsdom = new JSDOM();
const window = jsdom.window;
global.DOMParser = window.DOMParser;
global.XMLSerializer = window.XMLSerializer;

const args = process.argv;
if (args.includes('--help')) {

    console.log('xacro-parser <input-file> [--oldorder]');
    process.exit(0);

}

const inputFile = args[2];
if (!inputFile) {

    console.error('input file required');
    process.exit(1);

}

const inOrder = !args.includes('--oldorder');
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
