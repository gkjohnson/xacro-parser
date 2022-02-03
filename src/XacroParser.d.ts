export class XacroParser {

    rospackCommands?: { [key: string]: (...args:string[]) => string } | ((command: string, ...args: string[]) => string);
    localProperties?: boolean;
    inOrder?: boolean;
    workingPath?: string;
    requirePrefix?: boolean;

    parse(content: string): Promise<XMLDocument>;
    getFileContents(path: string): Promise<string>;

}
