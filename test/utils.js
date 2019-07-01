
function unformat(xml) {
    return xml
        .replace(/>\s+</g, '><')
        .replace(/<!--[\w\W]*?-->/g, '') // TODO: remove this
        .replace(/\s*xmlns:xacro=".*?"/g, '')
        .trim();
}

module.exports = { unformat };
