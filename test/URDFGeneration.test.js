// Tests from https://github.com/ros/xacro/blob/melodic-devel/test/test_xacro.py
/* global describe, it, expect, beforeEach, jest */
const { XacroParser } = require('../umd/XacroParser.js');
const { JSDOM } = require('jsdom');
const W3CXMLSerializer = require('w3c-xmlserializer');
const request = require('request');
const { unformat } = require('./utils.js');
const fs = require('fs');
const path = require('path');

jest.setTimeout(120000);

function getFileContents(p) {

    return new Promise((resolve, reject) => {

        request(p, (err, response, body) => {

            if (err) {
                reject(err);
            } else {
                resolve(body);
            }

        });

    });

};

function getLocalContents(p) {
    return fs.readFileSync(p, {encoding: 'utf8'});
}

beforeEach(() => {
    global.DOMParser = new JSDOM().window.DOMParser;
    global.XMLSerializer = W3CXMLSerializer.XMLSerializer.interface;
});

describe('XacroParser', () => {

    // TODO: The URL is now authentication blocked -- should download the data and host it in a github repo.
    it.skip('should parse the Robonaut 2 Xacro correctly.', async() => {

        const stem = 'https://gitlab.com/nasa-jsc-robotics/r2_description/raw/654f4f89ff8e802cb7f80c617f0d6dd04483f4b2/robots/';
        const r2b = stem + 'r2b.xacro';

        const parser = new XacroParser();
        parser.inOrder = false;
        parser.requirePrefix = false;
        parser.localProperties = false;
        parser.workingPath = stem;
        parser.getFileContents = getFileContents;

        const text = await parser.getFileContents(r2b);
        const result = await parser.parse(text);
        let serialized = new XMLSerializer().serializeToString(result);
        serialized = serialized.replace(/(rpy=".+?") (xyz=".+?")/g, (match, m1, m2) => `${ m2 } ${ m1 }`);

        let answer = await parser.getFileContents('https://raw.githubusercontent.com/gkjohnson/nasa-urdf-robots/master/r2_description/robots/r2b.urdf');
        answer = answer.replace(/(rpy=".+?") (xyz=".+?")/g, (match, m1, m2) => `${ m2 } ${ m1 }`);
        answer = answer.replace('<?xml version="1.0" ?>', '').trim();

        expect(unformat(serialized)).toEqual(unformat(answer));

    });

    it('should parse the Valkyrie Xacro correctly.', async() => {

        const valDescription = path.resolve(__dirname, './data/val_description/');
        const stem = path.resolve(valDescription, './model/robots/');
        const r2b = path.resolve(stem, './valkyrie_A.xacro');

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

        const text = await parser.getFileContents(r2b);
        const result = await parser.parse(text);
        let serialized = new XMLSerializer().serializeToString(result);
        serialized = serialized.replace(/&#x9;/g, ' '); // unicode tab html entity
        serialized = serialized.replace(/(rpy=".+?") (xyz=".+?")/g, (match, m1, m2) => `${ m2 } ${ m1 }`);
        serialized = serialized.replace(/-?(([0-9]+?\.[0-9]+)|([0-9]+))([eE]-?[0-9]+)?/g, num => parseFloat(num).toFixed(6));
        serialized = serialized.replace(/(Kp=".+?") (Kd=".+?") (Ki=".+?")/g, (match, kp, kd, ki) => `${ kd } ${ ki } ${ kp }`);

        let answer = await getFileContents(path.resolve(valDescription, './baseline/valkyrie_A.urdf'));
        answer = answer.replace(/(rpy=".+?") (xyz=".+?")/g, (match, m1, m2) => `${ m2 } ${ m1 }`);
        answer = answer.replace('<?xml version="1.0" ?>', '').trim();
        answer = answer.replace(/-?(([0-9]+?\.[0-9]+)|([0-9]+))([eE]-?[0-9]+)?/g, num => parseFloat(num).toFixed(6));
        answer = answer.replace(/(Kp=".+?") (Kd=".+?") (Ki=".+?")/g, (match, kp, kd, ki) => `${ kd } ${ ki } ${ kp }`);
        expect(unformat(serialized)).toEqual(unformat(answer));

    });

});
