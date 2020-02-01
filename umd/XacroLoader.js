(function (global, factory) {
    typeof exports === 'object' && typeof module !== 'undefined' ? module.exports = factory(require('./XacroParser.js')) :
    typeof define === 'function' && define.amd ? define(['./XacroParser.js'], factory) :
    (global.XacroLoader = factory(global.XacroParser_js));
}(this, (function (XacroParser_js) { 'use strict';

    function getUrlBase(url) {
        const tokens = url.split(/[\\/]/g);
        tokens.pop();
        if (tokens.length === 0) {
            return './';
        } else {
            return tokens.join('/') + '/';
        }
    }

    // XML Helpers
    // QuerySelectorAll that respects tag prefixes like 'xacro:'
    function getElementsWithName(node, name, res = []) {
        if (node.tagName === name) {
            res.push(node);
        }
        for (let i = 0, l = node.children.length; i < l; i++) {
            const child = node.children[i];
            getElementsWithName(child, name, res);
        }
        return res;
    }

    // Deep clone an xml node without the macro or property tags.
    function deepClone(node, stripPropsMacros) {
        const cloned = node.cloneNode();
        const childNodes = node.childNodes;
        for (let i = 0, l = childNodes.length; i < l; i++) {
            const child = childNodes[i];
            const tagName = child.tagName;
            if (!stripPropsMacros || (tagName !== 'xacro:property' && tagName !== 'xacro:macro')) {
                cloned.appendChild(deepClone(child, stripPropsMacros));
            }
        }
        return cloned;
    }

    // Takes an array of xml elements and removes the last elements that
    // are comments or newlines.
    function removeEndCommentsFromArray(arr) {
        while (arr.length > 0) {
            const el = arr[arr.length - 1];
            if (el.nodeType !== el.ELEMENT_NODE) {
                arr.pop();
            } else {
                break;
            }
        }
    }

    // Expression helpers
    function isOperator(str) {
        const regexp = /^[()/*+\-%|&=[\]]+$/;
        return regexp.test(str);
    }

    function isString(str) {
        const regexp = /^(('[^']*?')|("[^"]*?")|(`[^`]*?`))$/;
        return regexp.test(str);
    }

    // TODO: make this more robust
    function isNumber(str) {
        return !isNaN(parseFloat(str)) && !/[^0-9.eE-]/.test(str);
    }

    // TODO: this needs to tokenize numbers together
    function tokenize(str) {
        const regexp = /(('[^']*?')|("[^"]*?")|(`[^`]*?`)|([()/*+\-%|&=[\]]+))/g;
        return str
            .replace(regexp, m => ` ${ m } `)
            .trim()
            .split(/\s+/g);
    }

    function normalizeExpression(str) {
        // Remove any instances of "--" or "++" that might occur from negating a negative number
        // by adding a space that are not in a string.
        return str.replace(/[-+]{2,}/, val => {
            let positive = true;
            for (let i = 0, l = val.length; i < l; i++) {
                const operator = val[i];
                if (operator === '-') {
                    positive = !positive;
                }
            }

            return positive ? '+' : '-';
        });
    }

    // Property Set Helpers
    const PARENT_SCOPE = Symbol('parent');

    // merges a set of properties together into a single set retaining
    // the parent scope link as well.
    function mergePropertySets(...args) {
        const res = {};
        for (let i = 0, l = args.length; i < l; i++) {
            const obj = args[i];
            for (const key in obj) {
                res[key] = obj[key];
            }
            if (PARENT_SCOPE in obj) {
                res[PARENT_SCOPE] = obj[PARENT_SCOPE];
            }
        }
        return res;
    }

    // Copies a property set and creates a link to the original set as a parent scope
    function createNewPropertyScope(properties) {
        const res = mergePropertySets(properties);
        res[PARENT_SCOPE] = properties;
        return res;
    }

    class XacroLoader extends XacroParser_js.XacroParser {

        constructor() {

            super();
            this.fetchOptions = {};

        }

        load(url, onComplete, onError) {

            const workingPath = getUrlBase(url);
            if (this.workingPath === '') {

                this.workingPath = workingPath;

            }

            this
                .getFileContents(url)
                .then(text => {

                    this.parse(text, onComplete, onError);

                })
                .catch(e => {

                    if (onError) {

                        onError(e);

                    }

                });

        }

        parse(data, onComplete, onError) {

            super
                .parse(data)
                .then(onComplete)
                .catch(e => {

                    if (onError) {

                        onError(e);

                    }

                });

        }

        getFileContents(path) {

            return fetch(path, this.fetchOptions).then(res => res.text());

        }

    }

    return XacroLoader;

})));
//# sourceMappingURL=XacroLoader.js.map
