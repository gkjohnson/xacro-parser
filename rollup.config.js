const resolve = require('@rollup/plugin-node-resolve');
const path = require('path');
const inputPath = path.join(__dirname, `./src/index.js`);
const outputPath = path.join(__dirname, `./umd/index.cjs`);

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
