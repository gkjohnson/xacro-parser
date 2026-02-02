#!/usr/bin/env node

import fs from 'fs';
import path from 'path';
import { XacroParser } from '../src/XacroParser.js';
import { JSDOM } from 'jsdom';
import yargs from 'yargs';
import { hideBin } from 'yargs/helpers';
import * as YAML from 'yaml';

const argv = yargs( hideBin( process.argv ) )
	.positional( 'input', {
		type: 'string',
		description: 'The .xacro file to process.',
	} )
	.option( 'oldorder', {
		type: 'boolean',
		default: false,
		description: 'If enabled then the parser "inOrder" flag is disabled.',
	} )
	.option( 'package', {
		type: 'object',
		default: {},
		description: 'Set of package paths. Set a package path with "--package.<package name>=<package path>".',
	} )
	.option( 'arg', {
		type: 'object',
		default: {},
		description: 'Set of package ROS ecosystem args. Set an arg with "--arg.<arg name>=<arg value>".',
	} )
	.help()
	.argv;

const jsdom = new JSDOM();
const window = jsdom.window;
global.DOMParser = window.DOMParser;
global.XMLSerializer = window.XMLSerializer;

let inputFile = argv._[ 0 ];
if ( ! inputFile ) {

	console.error( 'input file required' );
	process.exit( 1 );

}

inputFile = path.join( process.cwd(), inputFile );

const inOrder = ! argv.oldorder;
const parser = new XacroParser();
parser.inOrder = inOrder;
parser.requirePrefix = inOrder;
parser.localProperties = inOrder;
parser.workingPath = path.dirname( inputFile );
parser.rospackCommands.arg = arg => {

	return argv.arg[ arg ];

};

parser.rospackCommands.find = pkg => {

	return path.join( process.cwd(), argv.package[ pkg ] || argv.package.default );

};

parser.getFileContents = p => {

	return fs.readFileSync( p, { encoding: 'utf8' } );

};

parser.expressionParser.functions.load_yaml = p => {

	const contents = fs.readFileSync( p, { encoding: 'utf-8' } );
	const result = YAML.parse( contents );
	return result;

};

parser
	.parse( parser.getFileContents( inputFile ) )
	.then( xml => {

		const serializer = new XMLSerializer();
		console.log( serializer.serializeToString( xml ) );

	} );
