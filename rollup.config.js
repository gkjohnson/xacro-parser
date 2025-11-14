import resolve from '@rollup/plugin-node-resolve';
import path from 'path';

const inputPath = path.join(import.meta.dirname, `./src/index.js`);
const outputPath = path.join(import.meta.dirname, `./umd/index.cjs`);

export default {

    input: inputPath,
    treeshake: false,
    plugins: [resolve()],

    output: {

        name: 'window',
        extend: true,
        format: 'umd',
        file: outputPath,
        sourcemap: true,

    },

};
