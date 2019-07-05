interface XacroLoaderOptions {

    rospackCommands?: { [key: string]: (...args:string[]) => string },
    localProperties?: boolean,
    inOrder?: boolean,
    fetchOptions?: object,
    workingPath?: string,
    requirePrefix?: boolean,

}

export default class XacroLoader {

    load(url: string, options?: XacroLoaderOptions): Promise<XMLDocument>;
    parse(content: string, options?: XacroLoaderOptions): Promise<XMLDocument>;

}
