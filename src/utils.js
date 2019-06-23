
export function getUrlBase(url) {

    const tokens = url.split(/[\\/]/g);
    tokens.pop();
    if (tokens.length === 0) return './';
    return tokens.join('/') + '/';

}
