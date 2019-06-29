import { LoadingManager, Object3D } from 'three';
import { URDFRobot } from './URDFClasses';

interface XacroLoaderOptions {

    rospackCommands?: { [key: string]: (...args:string[]) => string },
    localProperties?: boolean,
    inOrder?: boolean,
    fetchOptions?: object,
    workingPath?: string,
    requirePrefix?: boolean,

}

export default class URDFLoader {

    manager: LoadingManager;

    constructor(manager?: LoadingManager);
    load(url: string, onLoad: (result: XMLDocument) => void, options?: XacroLoaderOptions): void;
    parse(content: string, onLoad: (result: XMLDocument) => void, options?: XacroLoaderOptions): void;

}
