import { getUrlBase } from './utils.js';
import { XacroParser } from './XacroParser.js';

export default
class XacroLoader {

    /* Public API */
    load(url, onComplete, options) {

        const workingPath = getUrlBase(url);
        const parser = new XacroParser();
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

        const parser = new XacroParser();
        Object.assign(parser, options);
        parser.getFileContents = async function(path) {
            return (await fetch(path, options.fetchOptions)).text();
        };

        parser
            .parse(data)
            .then(xml => onLoad(xml));

    }

}
