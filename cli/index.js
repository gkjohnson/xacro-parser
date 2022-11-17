#!/usr/bin/env node

import fs from 'fs';
import path from 'path';
import { XacroParser } from '../src/XacroParser.js';
import { JSDOM } from 'jsdom';
import yargs from 'yargs';
import { hideBin } from 'yargs/helpers';

const argv = yargs(hideBin(process.argv))
    .positional('input', {
        type: 'string',
    })
    .option('oldorder', {
        type: 'boolean',
        default: false,
    })
    .option('package', {
        type: 'string',
        default: null,
    })
    .help()
    .argv;

const jsdom = new JSDOM();
const window = jsdom.window;
global.DOMParser = window.DOMParser;
global.XMLSerializer = window.XMLSerializer;

let inputFile = argv._[0];
if (!inputFile) {

    console.error('input file required');
    process.exit(1);

}

inputFile = path.join(process.cwd(), inputFile);

const inOrder = !argv.oldorder;
const parser = new XacroParser();
parser.inOrder = inOrder;
parser.requirePrefix = inOrder;
parser.localProperties = inOrder;
parser.workingPath = path.dirname(inputFile);
parser.rospackCommands.find = () => {

    return path.join(process.cwd(), argv.package);

};

parser.getFileContents = p => {

    return fs.readFileSync(p, { encoding: 'utf8' });

};

parser
    .parse(parser.getFileContents(inputFile))
    .then(xml => {

        const serializer = new XMLSerializer();
        console.log(serializer.serializeToString(xml));

    });
