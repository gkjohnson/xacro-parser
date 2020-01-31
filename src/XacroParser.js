import { getUrlBase } from './utils.js';

// TODO: Verify against ros-ran xacro files
// TODO: Report errors / warnings back to the client

const PARENT_SCOPE = Symbol('parent');
export class XacroParser {

    constructor() {
        this.inOrder = true;
        this.requirePrefix = true;
        this.localProperties = true;
        this.rospackCommands = {};
        this.workingPath = '';
    }

    async getFileContents(path) {
        throw new Error('XacroParser: getFileContents() not implemented.');
    }

    async parse(data) {

        /* Utilities */
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

        function mergeObjects(...args) {
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

        function createNewScope(properties) {
            const res = mergeObjects(properties);
            res[PARENT_SCOPE] = properties;
            return res;
        }

        // Deep clone an xml node without the macro or property tags.
        function deepClone(node, stripPropsMacros) {
            const res = node.cloneNode();
            const childNodes = node.childNodes;
            for (let i = 0, l = childNodes.length; i < l; i++) {
                const c = childNodes[i];
                const tagName = c.tagName;
                if (!stripPropsMacros || (tagName !== 'xacro:property' && tagName !== 'xacro:macro')) {
                    res.appendChild(deepClone(c, stripPropsMacros));
                }
            }
            return res;
        }

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

        /* Evaluation */
        // Evaluate expressions and rospack commands in attribute text
        // TODO: expressions can basically be any python expression
        // TODO: support proper expression evaluation without Function or eval
        function evaluateAttribute(str, properties) {

            // recursively unpack parameters
            function unpackParams(str, properties) {
                if (typeof str === 'number') return str;

                const res = str.replace(/(\$?\$\([^)]+\))|(\$?\${[^}]+})/g, match => {

                    if (/^\$\$/.test(match)) return match.substring(1);

                    const isRospackCommand = /^\$\(/.test(match);
                    const contents = match.substring(2, match.length - 1);
                    if (isRospackCommand) {

                        const command = unpackParams(contents, properties);
                        const tokens = command.split(/\s+/g);
                        const stem = tokens.shift();

                        if (stem in rospackCommands) {
                            return rospackCommands[stem](...tokens);
                        } else {
                            throw new Error(`XacroParser: Cannot run rospack command "${ contents }"`);
                        }

                    } else {
                        if (stack.includes(contents)) {
                            throw new Error(
                                `XacroParser: Cannot evaluate infinitely recursive expression: ${
                                    stack.join(' > ')
                                } > ${
                                    contents
                                }`
                            );
                        }

                        stack.push(contents);

                        const operators = /[()/*+\-%|&=]+/g;
                        const expr = contents
                            .replace(operators, m => ` ${ m } `)
                            .trim()
                            .split(/\s+/g)
                            .map(t => {
                                operators.lastIndex = 0;
                                if (operators.test(t)) return t;
                                if (!isNaN(parseFloat(t))) return t;
                                if (/^'.*?'$/.test(t)) return t;
                                if (/^".*?"$/.test(t)) return t;

                                if (t in properties) {
                                    const arg = unpackParams(properties[t], properties);
                                    if (isNaN(parseFloat(arg)) || /[^0-9.eE-]/.test(arg)) {
                                        return `"${ arg.toString().replace(/\\/g, '\\\\').replace(/"/g, '\\"') }"`;
                                    } else {
                                        return arg;
                                    }
                                } else {
                                    throw new Error(
                                        `XacroParser: Missing parameter "${ t }".`
                                    );
                                }
                            })
                            .join('');

                        stack.pop();
                        if (isNaN(parseFloat(expr)) || /[^0-9.eE-]/.test(expr)) {

                            // Remove any instances of "--" or "++" that might occur from negating a negative number
                            // that are not in a string.
                            const cleanExpr = expr.replace(/[+-]{2}(?=([^"]*"[^"]*")*[^"]*$)/g, (m, rest) => ` ${ m[0] } ${ m[1] } ${ rest || '' }`);

                            // TODO: Remove the potentially unsafe use of Function
                            return (new Function(`return ${ cleanExpr };`))(); // eslint-disable-line no-new-func

                        } else {
                            return expr;
                        }
                    }

                });

                return res;

            }

            const stack = [];
            const allProps = mergeObjects(globalProperties, properties);
            try {
                return unpackParams(str, allProps);
            } catch (e) {
                throw new Error(`XacroParser: Failed to process expression "${ str }". \n` + e.message);
            }

        }

        // Evaluate the given node as a macro
        async function evaluateMacro(node, properties, macros, resultsList) {

            // Find the macro
            const macroName = node.tagName.replace(/^xacro:/, '');
            const macro = macros[macroName];

            if (!macro) {
                throw new Error(`XacroParser: Cannot find macro "${ macroName }"`);
            }

            // Copy the properties and macros so we can modify them with
            // macro input fields and local macro definitions.
            const ogProperties = properties;
            const ogMacros = macros;
            properties = createNewScope(properties);
            macros = mergeObjects(macros);

            // Modify the properties with macro param inputs
            let children = [];
            for (const c of node.children) {
                await processNode(c, ogProperties, ogMacros, children);
            }
            children = children.filter(c => c.nodeType === c.ELEMENT_NODE);

            let blockCount = 0;
            for (const p in macro.params) {
                const param = macro.params[p];
                if (node.hasAttribute(p)) {
                    properties[p] = evaluateAttribute(node.getAttribute(p), ogProperties);
                } else if (param.type === 'BLOCK') {
                    properties[p] = [children[blockCount]];
                    blockCount++;
                } else if (param.type === 'MULTI_BLOCK') {
                    properties[p] = [...children.filter(c => c.tagName === p)[0].childNodes];
                } else {
                    properties[p] = evaluateAttribute(macro.params[p].def, ogProperties);
                }
            }

            // Expand the macro
            const macroChildren = [...macro.node.childNodes];
            for (const c of macroChildren) {
                const nodes = [];
                await processNode(c, properties, macros, nodes);
                resultsList.push(...nodes);
            }
        }

        /* Parsing */
        // Conver the params into an object representation
        function parseMacroParam(param) {
            const obj = {};

            // Save the type of parameter
            // - two asterisks means an element expands input multiple
            // - one asterisk means copy the first elemnt
            // - no asterisks means value param
            if (/^\*\*/.test(param)) {
                obj.type = 'MULTI_BLOCK';
            } else if (/^\*/.test(param)) {
                obj.type = 'BLOCK';
            } else {
                obj.type = 'PARAM';
            }

            // strip the asterisks
            param = param.replace(/^\*{1,2}/g, '');

            // Check if a default value is provided
            if (/:=/.test(param)) {
                const [name, def] = param.split(':=');

                // TODO: Support caret and default syntax
                // TODO: is there any difference between the := and = syntax?
                if (/^\^/.test(def) || /\|/.test(def)) {
                    throw new Error(`XacroParser: ROS Jade pass-through notation not supported in macro defaults: ${ def }`);
                }

                obj.name = name;
                obj.def = def;
            } else {
                obj.name = param;
                obj.def = null;
            }

            return obj;
        }

        // Parse a xacro:macro tag
        function parseMacro(node) {
            // get attributes
            const name = node.getAttribute('name').replace(/^xacro:/, '');
            const params = node.getAttribute('params');

            // parse params
            const inputMap = {};
            if (params) {
                const inputs = params
                    .trim()
                    .split(/\s+/g)
                    .map(s => parseMacroParam(s));
                inputs.forEach(inp => {
                    inputMap[inp.name] = inp;
                });
            }

            return {
                name,
                node: deepClone(node, false),
                params: inputMap,
            };
        }

        // Recursively process and expand a node
        async function processNode(node, properties, macros, resultsList = []) {
            if (node.nodeType !== node.ELEMENT_NODE) {
                const res = node.cloneNode();
                res.textContent = evaluateAttribute(res.textContent, properties);
                resultsList.push(res);
                return;
            }

            let tagName = node.tagName.toLowerCase();
            if (!requirePrefix) {
                switch (tagName) {

                    case 'property':
                    case 'macro':
                    case 'insert_block':
                    case 'if':
                    case 'unless':
                    case 'include':
                    case 'element':
                    case 'attribute':
                        tagName = `xacro:${ tagName }`;
                        break;
                    default:
                        if (tagName in macros) {
                            tagName = `xacro:${ tagName }`;
                        }
                        break;

                }
            }

            switch (tagName) {

                case 'xacro:property': {
                    removeEndCommentsFromArray(resultsList);

                    const name = node.getAttribute('name');

                    let value;
                    if (node.hasAttribute('value')) {
                        value = node.getAttribute('value');
                    } else if (node.hasAttribute('default')) {
                        value = node.getAttribute('default');
                    } else {
                        const childNodes = [...node.childNodes];
                        value = [];
                        for (const c of childNodes) {
                            value.push(deepClone(c, false));
                        }
                    }

                    let scope = 'global';
                    if (localProperties) {
                        scope = node.getAttribute('scope') || 'local';
                    }

                    // Emulated behavior here
                    // https://github.com/ros/xacro/blob/melodic-devel/src/xacro/__init__.py#L565
                    if (scope !== 'local') {
                        value = evaluateAttribute(value, properties);
                    }

                    if (scope === 'global') {
                        globalProperties[name] = value;
                    } else if (scope === 'parent') {
                        properties[PARENT_SCOPE][name] = value;
                    } else {
                        properties[name] = value;
                    }

                    break;
                }
                case 'xacro:macro': {
                    removeEndCommentsFromArray(resultsList);

                    const macro = parseMacro(node);
                    macros[macro.name] = macro;
                    break;
                }
                case 'xacro:insert_block': {
                    removeEndCommentsFromArray(resultsList);

                    const name = node.getAttribute('name');
                    const nodes = properties[name];

                    for (const c of nodes) {
                        await processNode(c, properties, macros, resultsList);
                    }
                    return;
                }
                case 'xacro:if':
                case 'xacro:unless': {
                    removeEndCommentsFromArray(resultsList);

                    const value = evaluateAttribute(node.getAttribute('value'), properties);
                    let bool = null;
                    if (!isNaN(parseFloat(value))) {
                        bool = !!parseFloat(value);
                    } else if (value === 'true' || value === 'false') {
                        bool = value === 'true';
                    } else {
                        bool = value;
                    }

                    if (tagName === 'xacro:unless') {
                        bool = !bool;
                    }

                    if (bool) {
                        const childNodes = [...node.childNodes];
                        for (const c of childNodes) {
                            await processNode(c, properties, macros, resultsList);
                        }
                    }
                    return;
                }
                case 'xacro:include': {
                    removeEndCommentsFromArray(resultsList);

                    if (node.hasAttribute('ns')) {
                        throw new Error('XacroParser: xacro:include name spaces not supported.');
                    }
                    const filename = evaluateAttribute(node.getAttribute('filename'), properties);
                    const isAbsolute = /^[/\\]/.test(filename) || /^[a-zA-Z]+:[/\\]/.test(filename);
                    const filePath = isAbsolute ? filename : currWorkingPath + filename;

                    const prevWorkingPath = currWorkingPath;
                    currWorkingPath = getUrlBase(filePath);

                    const includeContent = await loadInclude(filePath);
                    const childNodes = [...includeContent.children[0].childNodes];
                    for (const c of childNodes) {
                        await processNode(c, properties, macros, resultsList);
                    }

                    currWorkingPath = prevWorkingPath;
                    return;
                }
                case 'xacro:attribute':
                case 'xacro:element':
                    removeEndCommentsFromArray(resultsList);
                    throw new Error(`XacroParser: ${ tagName } tags not supported.`);
                default: {
                    // TODO: check if there's a 'call' attribute here which indicates that
                    // a macro should be invoked?
                    if (/^xacro:/.test(tagName) || tagName in macros) {
                        removeEndCommentsFromArray(resultsList);

                        return evaluateMacro(node, properties, macros, resultsList);
                    } else {

                        const res = node.cloneNode();
                        for (let i = 0, l = res.attributes.length; i < l; i++) {
                            const attr = res.attributes[i];
                            const value = evaluateAttribute(attr.value, properties);
                            res.setAttribute(attr.name, value);
                        }

                        const childNodes = [...node.childNodes];
                        const resultChildren = [];
                        for (let i = 0, l = childNodes.length; i < l; i++) {
                            await processNode(childNodes[i], properties, macros, resultChildren);
                        }
                        resultChildren.forEach(c => res.appendChild(c));
                        resultsList.push(res);
                    }
                }

            }
        }

        // Process all property and macro tags into the objects
        async function gatherPropertiesAndMacros(el, properties, macros) {
            const propertyEl = getElementsWithName(el, 'xacro:property');
            if (!requirePrefix) {
                propertyEl.push(...getElementsWithName(el, 'property'));
            }
            for (const el of propertyEl) {
                await processNode(el, properties, macros);
            }

            const macroEl = getElementsWithName(el, 'xacro:macro');
            if (!requirePrefix) {
                macroEl.push(...getElementsWithName(el, 'macro'));
            }
            for (const el of macroEl) {
                await processNode(el, properties, macros);
            }
        }

        // Process a document node with a new property and macro scope
        async function processXacro(xacro, properties, macros) {
            const res = xacro.cloneNode();
            for (let i = 0, l = xacro.children.length; i < l; i++) {
                const child = [];
                await processNode(xacro.children[i], properties, macros, child);

                const root = child[0];
                root.removeAttribute('xmlns:xacro');
                res.appendChild(root);
            }
            return res;
        }

        async function loadInclude(path) {

            try {
                const text = await scope.getFileContents(path);
                return new DOMParser().parseFromString(text, 'text/xml');
            } catch (e) {
                throw new Error(`XacroParser: Could not load included file: ${ path }`);
            }

        }

        async function loadIncludes(xacro, workingPath, results = []) {

            const includeEl = getElementsWithName(xacro, 'xacro:include');
            if (!requirePrefix) {
                includeEl.push(...getElementsWithName(xacro, 'include'));
            }

            const promises = includeEl.map(el => {
                // TODO: Handle namespaces on the include.
                if (el.hasAttribute('ns')) {
                    throw new Error('XacroParser: xacro:include name spaces not supported.');
                }

                const filename = el.getAttribute('filename');
                const namespace = el.getAttribute('ns') || null;
                const isAbsolute = /^[/\\]/.test(filename) || /^[a-zA-Z]+:[/\\]/.test(filename);
                const filePath = isAbsolute ? filename : workingPath + filename;
                const pr = loadInclude(filePath)
                    .then(content => {
                        results.push({ filename, namespace, content });

                        const relPath = getUrlBase(filePath);
                        return loadIncludes(content, relPath, results);
                    });
                return pr;
            });

            await Promise.all(promises);
            return results;
        }

        // TODO: Provide a default "arg" command function that defaults to
        // xacro:arg fields.
        const scope = this;
        const inOrder = this.inOrder;
        const workingPath = this.workingPath + (!/[\\/]$/.test(this.workingPath) ? '/' : '');
        const requirePrefix = this.requirePrefix;
        const rospackCommands = this.rospackCommands;
        const globalMacros = {};
        const includeMap = {};
        const globalProperties = { True: 1, False: 0 };
        globalProperties[PARENT_SCOPE] = globalProperties;

        let localProperties = this.localProperties;
        let currWorkingPath = workingPath;
        let content = new DOMParser().parseFromString(data, 'text/xml');

        if (localProperties && !inOrder) {
            console.warn('XacroParser: Implicitly setting "localProperties" option to false because "inOrder" is false.');
            localProperties = false;
        }

        let inOrderPromise = null;
        if (!inOrder) {
            inOrderPromise = (async function() {
                await gatherPropertiesAndMacros(content, globalProperties, globalMacros);
                content = deepClone(content, true);

                return loadIncludes(content, workingPath)
                    .then(arr => {
                        arr.forEach(inc => {
                            // TODO: handle namespaces here when rolling up properties and macros
                            gatherPropertiesAndMacros(inc.content, globalProperties, globalMacros);
                            inc.content = deepClone(inc.content, true);
                            includeMap[inc.filename] = inc.content;
                        });
                    });
            })();
        } else {
            inOrderPromise = Promise.resolve();
        }

        await inOrderPromise;
        return processXacro(content, globalProperties, globalMacros);
    }

}
