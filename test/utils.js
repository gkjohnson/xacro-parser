
function unformat(xml) {
    return xml
        .replace(/>\s+</g, '><')
        .replace(/\s*xmlns:xacro=".*?"/g, '')
        .replace(/[+-]?\d+[eE\-+.\d]+\d+?/g, num => parseFloat(num).toFixed(6))
        .trim();
}

module.exports = { unformat };
