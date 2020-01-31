// Tests from https://github.com/ros/xacro/blob/melodic-devel/test/test_xacro.py
/* global describe, it, expect, beforeEach */
const { XacroParser } = require('../umd/XacroParser.js');
const { JSDOM } = require('jsdom');
const W3CXMLSerializer = require('w3c-xmlserializer');
const { unformat } = require('./utils.js');
const fs = require('fs');
const path = require('path');

function getLocalContents(p) {
    return fs.readFileSync(p, {encoding: 'utf8'});
}

beforeEach(() => {
    global.DOMParser = new JSDOM().window.DOMParser;
    global.XMLSerializer = W3CXMLSerializer.XMLSerializer.interface;
});

describe('XacroParser', () => {

    it('should parse the Robonaut 2 Xacro correctly.', async() => {

        const r2Description = path.resolve(__dirname, './data/r2_description/');
        const stem = path.resolve(r2Description, './robots/');
        const r2b = path.resolve(stem, 'r2b.xacro');

        const parser = new XacroParser();
        parser.inOrder = false;
        parser.requirePrefix = false;
        parser.localProperties = false;
        parser.workingPath = stem;
        parser.getFileContents = getLocalContents;

        const text = await parser.getFileContents(r2b);
        const result = await parser.parse(text);
        let serialized = new XMLSerializer().serializeToString(result);
        serialized = serialized.replace(/(rpy=".+?") (xyz=".+?")/g, (match, m1, m2) => `${ m2 } ${ m1 }`);

        let answer = await getLocalContents(path.resolve(r2Description, './baseline/r2b.urdf'));
        answer = answer.replace(/(rpy=".+?") (xyz=".+?")/g, (match, m1, m2) => `${ m2 } ${ m1 }`);
        answer = answer.replace('<?xml version="1.0" ?>', '').trim();

        expect(unformat(serialized)).toEqual(unformat(answer));

    });

    it('should parse the Valkyrie Xacro correctly.', async() => {

        const valDescription = path.resolve(__dirname, './data/val_description/');
        const stem = path.resolve(valDescription, './model/robots/');
        const rootFile = path.resolve(stem, './valkyrie_A.xacro');

        const parser = new XacroParser();
        parser.rospackCommands = {
            find: pkg => {
                if (pkg === 'val_description') {
                    return valDescription;
                } else {
                    throw new Error();
                }
            },
        };
        parser.workingPath = stem;
        parser.getFileContents = getLocalContents;

        const text = await parser.getFileContents(rootFile);
        const result = await parser.parse(text);
        let serialized = new XMLSerializer().serializeToString(result);
        serialized = serialized.replace(/&#x9;/g, ' '); // unicode tab html entity
        serialized = serialized.replace(/(rpy=".+?") (xyz=".+?")/g, (match, m1, m2) => `${ m2 } ${ m1 }`);
        serialized = serialized.replace(/-?(([0-9]+?\.[0-9]+)|([0-9]+))([eE]-?[0-9]+)?/g, num => parseFloat(num).toFixed(6));
        serialized = serialized.replace(/(Kp=".+?") (Kd=".+?") (Ki=".+?")/g, (match, kp, kd, ki) => `${ kd } ${ ki } ${ kp }`);

        let answer = await getLocalContents(path.resolve(valDescription, './baseline/valkyrie_A.urdf'));
        answer = answer.replace(/(rpy=".+?") (xyz=".+?")/g, (match, m1, m2) => `${ m2 } ${ m1 }`);
        answer = answer.replace('<?xml version="1.0" ?>', '').trim();
        answer = answer.replace(/-?(([0-9]+?\.[0-9]+)|([0-9]+))([eE]-?[0-9]+)?/g, num => parseFloat(num).toFixed(6));
        answer = answer.replace(/(Kp=".+?") (Kd=".+?") (Ki=".+?")/g, (match, kp, kd, ki) => `${ kd } ${ ki } ${ kp }`);
        expect(unformat(serialized)).toEqual(unformat(answer));

    });

});
