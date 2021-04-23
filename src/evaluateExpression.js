import { isNumber } from './utils.js';
import { Parser } from 'expr-eval';

const parser = new Parser();

parser.unaryOps = {
    '-': parser.unaryOps['-'],
    '+': parser.unaryOps['+'],
    '!': parser.unaryOps['not'],
    'not': parser.unaryOps['not'],
};

parser.functions = {
    sin: Math.sin,
    cos: Math.cos,
    tan: Math.tan,
    asin: Math.asin,
    acos: Math.acos,
    atan: Math.atan,
    log: Math.log,
    atan2: Math.atan2,
    pow: Math.pow,
    radians: (degrees) => { return degrees * (Math.PI / 180); },
    degrees: (radians) => { return radians * (180 / Math.PI); },
};

parser.binaryOps = {
    ...parser.binaryOps,
    '+': (a, b) => {
        if (isNumber(a)) {
            a = Number(a);
        }

        if (isNumber(b)) {
            b = Number(b);
        }

        return a + b;
    },
    'in': (a, b) => {
        if (Array.isArray(b)) {
            return b.includes(a);
        } else if (typeof b === 'string') {
            return b.includes(a);
        } else {
            return a in b;
        }
    },
    '||': (a, b) => Boolean(a || b),
    '&&': (a, b) => Boolean(a || b),
};

parser.consts = {
    ...parser.consts,
    pi: Math.PI,
    e: Math.E,
};

export function evaluateExpression(expr) {
    return parser.evaluate(expr);
}
