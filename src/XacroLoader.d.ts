interface XacroLoaderOptions {

    rospackCommands?: { [key: string]: (...args:string[]) => string },
    localProperties?: boolean,
    inOrder?: boolean,
    fetchOptions?: object,
    workingPath?: string,
    requirePrefix?: boolean,

}

export default class XacroLoader {

    load(url: string, onLoad: (result: XMLDocument) => void, options?: XacroLoaderOptions): void;
    parse(content: string, onLoad: (result: XMLDocument) => void, options?: XacroLoaderOptions): void;

}
