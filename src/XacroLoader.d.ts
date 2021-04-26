interface XacroLoaderOptions {

    rospackCommands?: { [key: string]: (...args:string[]) => string },
    localProperties?: boolean,
    inOrder?: boolean,
    fetchOptions?: object,
    workingPath?: string,
    requirePrefix?: boolean,

}

export class XacroLoader {

    load(url: string, onLoad: (xml: XMLDocument) => void, options?: XacroLoaderOptions): Promise<XMLDocument>;
    parse(content: string, onLoad: (xml: XMLDocument) => void, options?: XacroLoaderOptions): Promise<XMLDocument>;

}
