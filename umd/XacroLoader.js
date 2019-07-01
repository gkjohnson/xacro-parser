(function (global, factory) {
    typeof exports === 'object' && typeof module !== 'undefined' ? module.exports = factory(require('./XacroParser.js')) :
    typeof define === 'function' && define.amd ? define(['./XacroParser.js'], factory) :
    (global.XacroLoader = factory(global.XacroParser_js));
}(this, (function (XacroParser_js) { 'use strict';

    function getUrlBase(url) {

        const tokens = url.split(/[\\/]/g);
        tokens.pop();
        if (tokens.length === 0) return './';
        return tokens.join('/') + '/';

    }

    class XacroLoader {

        /* Public API */
        load(url, onComplete, options) {

            const workingPath = getUrlBase(url);
            const parser = new XacroParser_js.XacroParser();
            Object.assign(parser, { workingPath }, options);
            parser.getFileContents = async function(path) {
                return (await fetch(path, options.fetchOptions)).text();
            };

            parser
                .getFileContents(url)
                .then(text => parser.parse(text))
                .then(xml => onComplete(xml));

        }

        parse(data, onLoad, options = {}) {

            const parser = new XacroParser_js.XacroParser();
            Object.assign(parser, options);
            parser.getFileContents = async function(path) {
                return (await fetch(path, options.fetchOptions)).text();
            };

            parser
                .parse(data)
                .then(xml => onLoad(xml));

        }

    }

    return XacroLoader;

})));
//# sourceMappingURL=XacroLoader.js.map
