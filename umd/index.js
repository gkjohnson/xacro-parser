(function (global, factory) {
    typeof exports === 'object' && typeof module !== 'undefined' ? factory(exports) :
    typeof define === 'function' && define.amd ? define(['exports'], factory) :
    (global = global || self, factory(global.window = global.window || {}));
}(this, (function (exports) { 'use strict';

    function getUrlBase(url) {
        const tokens = url.split(/[\\/]/g);
        tokens.pop();
        if (tokens.length === 0) {
            return './';
        } else {
            return tokens.join('/') + '/';
        }
    }

    // XML Helpers
    // QuerySelectorAll that respects tag prefixes like 'xacro:'
    function getElementsWithName(node, name, res = []) {
        if (node.tagName === name) {
            res.push(node);
        }
        for (let i = 0, l = node.children.length; i < l; i++) {
            const child = node.children[i];
            getElementsWithName(child, name, res);
        }
        return res;
    }

    // Deep clone an xml node without the macro or property tags.
    function deepClone(node, stripPropsMacros) {
        const cloned = node.cloneNode();
        const childNodes = node.childNodes;
        for (let i = 0, l = childNodes.length; i < l; i++) {
            const child = childNodes[i];
            const tagName = child.tagName;
            if (!stripPropsMacros || (tagName !== 'xacro:property' && tagName !== 'xacro:macro')) {
                cloned.appendChild(deepClone(child, stripPropsMacros));
            }
        }
        return cloned;
    }

    // Takes an array of xml elements and removes the last elements that
    // are comments or newlines.
    function removeEndCommentsFromArray(arr) {
        while (arr.length > 0) {
            const el = arr[arr.length - 1];
            if (el.nodeType !== el.ELEMENT_NODE) {
                arr.pop();
            } else {
                break;
            }
        }
    }

    // Expression helpers
    function isOperator(str) {
        const regexp = /^[()/*+\-%|&=[\]]+$/;
        return regexp.test(str);
    }

    function isString(str) {
        const regexp = /^(('[^']*?')|("[^"]*?")|(`[^`]*?`))$/;
        return regexp.test(str);
    }

    // TODO: make this more robust
    function isNumber(str) {
        return !isNaN(parseFloat(str)) && !/[^0-9.eE-]/.test(str);
    }

    // TODO: this needs to tokenize numbers together
    function tokenize(str) {
        const regexp = /(('[^']*?')|("[^"]*?")|(`[^`]*?`)|([()/*+\-%|&=[\]]+))/g;
        return str
            .replace(regexp, m => ` ${ m } `)
            .trim()
            .split(/\s+/g);
    }

    function normalizeExpression(str) {
        // Remove any instances of "--" or "++" that might occur from negating a negative number
        // by adding a space that are not in a string.
        return str.replace(/[-+]{2,}/, val => {
            let positive = true;
            for (let i = 0, l = val.length; i < l; i++) {
                const operator = val[i];
                if (operator === '-') {
                    positive = !positive;
                }
            }

            return positive ? '+' : '-';
        });
    }

    // Property Set Helpers
    const PARENT_SCOPE = Symbol('parent');

    // merges a set of properties together into a single set retaining
    // the parent scope link as well.
    function mergePropertySets(...args) {
        const res = {};
        for (let i = 0, l = args.length; i < l; i++) {
            const obj = args[i];
            for (const key in obj) {
                res[key] = obj[key];
            }
            if (PARENT_SCOPE in obj) {
                res[PARENT_SCOPE] = obj[PARENT_SCOPE];
            }
        }
        return res;
    }

    // Copies a property set and creates a link to the original set as a parent scope
    function createNewPropertyScope(properties) {
        const res = mergePropertySets(properties);
        res[PARENT_SCOPE] = properties;
        return res;
    }

    var INUMBER = 'INUMBER';
    var IOP1 = 'IOP1';
    var IOP2 = 'IOP2';
    var IOP3 = 'IOP3';
    var IVAR = 'IVAR';
    var IVARNAME = 'IVARNAME';
    var IFUNCALL = 'IFUNCALL';
    var IFUNDEF = 'IFUNDEF';
    var IEXPR = 'IEXPR';
    var IEXPREVAL = 'IEXPREVAL';
    var IMEMBER = 'IMEMBER';
    var IENDSTATEMENT = 'IENDSTATEMENT';
    var IARRAY = 'IARRAY';

    function Instruction(type, value) {
      this.type = type;
      this.value = (value !== undefined && value !== null) ? value : 0;
    }

    Instruction.prototype.toString = function () {
      switch (this.type) {
        case INUMBER:
        case IOP1:
        case IOP2:
        case IOP3:
        case IVAR:
        case IVARNAME:
        case IENDSTATEMENT:
          return this.value;
        case IFUNCALL:
          return 'CALL ' + this.value;
        case IFUNDEF:
          return 'DEF ' + this.value;
        case IARRAY:
          return 'ARRAY ' + this.value;
        case IMEMBER:
          return '.' + this.value;
        default:
          return 'Invalid Instruction';
      }
    };

    function unaryInstruction(value) {
      return new Instruction(IOP1, value);
    }

    function binaryInstruction(value) {
      return new Instruction(IOP2, value);
    }

    function ternaryInstruction(value) {
      return new Instruction(IOP3, value);
    }

    function simplify(tokens, unaryOps, binaryOps, ternaryOps, values) {
      var nstack = [];
      var newexpression = [];
      var n1, n2, n3;
      var f;
      for (var i = 0; i < tokens.length; i++) {
        var item = tokens[i];
        var type = item.type;
        if (type === INUMBER || type === IVARNAME) {
          if (Array.isArray(item.value)) {
            nstack.push.apply(nstack, simplify(item.value.map(function (x) {
              return new Instruction(INUMBER, x);
            }).concat(new Instruction(IARRAY, item.value.length)), unaryOps, binaryOps, ternaryOps, values));
          } else {
            nstack.push(item);
          }
        } else if (type === IVAR && values.hasOwnProperty(item.value)) {
          item = new Instruction(INUMBER, values[item.value]);
          nstack.push(item);
        } else if (type === IOP2 && nstack.length > 1) {
          n2 = nstack.pop();
          n1 = nstack.pop();
          f = binaryOps[item.value];
          item = new Instruction(INUMBER, f(n1.value, n2.value));
          nstack.push(item);
        } else if (type === IOP3 && nstack.length > 2) {
          n3 = nstack.pop();
          n2 = nstack.pop();
          n1 = nstack.pop();
          if (item.value === '?') {
            nstack.push(n1.value ? n2.value : n3.value);
          } else {
            f = ternaryOps[item.value];
            item = new Instruction(INUMBER, f(n1.value, n2.value, n3.value));
            nstack.push(item);
          }
        } else if (type === IOP1 && nstack.length > 0) {
          n1 = nstack.pop();
          f = unaryOps[item.value];
          item = new Instruction(INUMBER, f(n1.value));
          nstack.push(item);
        } else if (type === IEXPR) {
          while (nstack.length > 0) {
            newexpression.push(nstack.shift());
          }
          newexpression.push(new Instruction(IEXPR, simplify(item.value, unaryOps, binaryOps, ternaryOps, values)));
        } else if (type === IMEMBER && nstack.length > 0) {
          n1 = nstack.pop();
          nstack.push(new Instruction(INUMBER, n1.value[item.value]));
        } /* else if (type === IARRAY && nstack.length >= item.value) {
          var length = item.value;
          while (length-- > 0) {
            newexpression.push(nstack.pop());
          }
          newexpression.push(new Instruction(IARRAY, item.value));
        } */ else {
          while (nstack.length > 0) {
            newexpression.push(nstack.shift());
          }
          newexpression.push(item);
        }
      }
      while (nstack.length > 0) {
        newexpression.push(nstack.shift());
      }
      return newexpression;
    }

    function substitute(tokens, variable, expr) {
      var newexpression = [];
      for (var i = 0; i < tokens.length; i++) {
        var item = tokens[i];
        var type = item.type;
        if (type === IVAR && item.value === variable) {
          for (var j = 0; j < expr.tokens.length; j++) {
            var expritem = expr.tokens[j];
            var replitem;
            if (expritem.type === IOP1) {
              replitem = unaryInstruction(expritem.value);
            } else if (expritem.type === IOP2) {
              replitem = binaryInstruction(expritem.value);
            } else if (expritem.type === IOP3) {
              replitem = ternaryInstruction(expritem.value);
            } else {
              replitem = new Instruction(expritem.type, expritem.value);
            }
            newexpression.push(replitem);
          }
        } else if (type === IEXPR) {
          newexpression.push(new Instruction(IEXPR, substitute(item.value, variable, expr)));
        } else {
          newexpression.push(item);
        }
      }
      return newexpression;
    }

    function evaluate(tokens, expr, values) {
      var nstack = [];
      var n1, n2, n3;
      var f, args, argCount;

      if (isExpressionEvaluator(tokens)) {
        return resolveExpression(tokens, values);
      }

      var numTokens = tokens.length;

      for (var i = 0; i < numTokens; i++) {
        var item = tokens[i];
        var type = item.type;
        if (type === INUMBER || type === IVARNAME) {
          nstack.push(item.value);
        } else if (type === IOP2) {
          n2 = nstack.pop();
          n1 = nstack.pop();
          if (item.value === 'and') {
            nstack.push(n1 ? !!evaluate(n2, expr, values) : false);
          } else if (item.value === 'or') {
            nstack.push(n1 ? true : !!evaluate(n2, expr, values));
          } else if (item.value === '=') {
            f = expr.binaryOps[item.value];
            nstack.push(f(n1, evaluate(n2, expr, values), values));
          } else {
            f = expr.binaryOps[item.value];
            nstack.push(f(resolveExpression(n1, values), resolveExpression(n2, values)));
          }
        } else if (type === IOP3) {
          n3 = nstack.pop();
          n2 = nstack.pop();
          n1 = nstack.pop();
          if (item.value === '?') {
            nstack.push(evaluate(n1 ? n2 : n3, expr, values));
          } else {
            f = expr.ternaryOps[item.value];
            nstack.push(f(resolveExpression(n1, values), resolveExpression(n2, values), resolveExpression(n3, values)));
          }
        } else if (type === IVAR) {
          if (item.value in expr.functions) {
            nstack.push(expr.functions[item.value]);
          } else if (item.value in expr.unaryOps && expr.parser.isOperatorEnabled(item.value)) {
            nstack.push(expr.unaryOps[item.value]);
          } else {
            var v = values[item.value];
            if (v !== undefined) {
              nstack.push(v);
            } else {
              throw new Error('undefined variable: ' + item.value);
            }
          }
        } else if (type === IOP1) {
          n1 = nstack.pop();
          f = expr.unaryOps[item.value];
          nstack.push(f(resolveExpression(n1, values)));
        } else if (type === IFUNCALL) {
          argCount = item.value;
          args = [];
          while (argCount-- > 0) {
            args.unshift(resolveExpression(nstack.pop(), values));
          }
          f = nstack.pop();
          if (f.apply && f.call) {
            nstack.push(f.apply(undefined, args));
          } else {
            throw new Error(f + ' is not a function');
          }
        } else if (type === IFUNDEF) {
          // Create closure to keep references to arguments and expression
          nstack.push((function () {
            var n2 = nstack.pop();
            var args = [];
            var argCount = item.value;
            while (argCount-- > 0) {
              args.unshift(nstack.pop());
            }
            var n1 = nstack.pop();
            var f = function () {
              var scope = Object.assign({}, values);
              for (var i = 0, len = args.length; i < len; i++) {
                scope[args[i]] = arguments[i];
              }
              return evaluate(n2, expr, scope);
            };
            // f.name = n1
            Object.defineProperty(f, 'name', {
              value: n1,
              writable: false
            });
            values[n1] = f;
            return f;
          })());
        } else if (type === IEXPR) {
          nstack.push(createExpressionEvaluator(item, expr));
        } else if (type === IEXPREVAL) {
          nstack.push(item);
        } else if (type === IMEMBER) {
          n1 = nstack.pop();
          nstack.push(n1[item.value]);
        } else if (type === IENDSTATEMENT) {
          nstack.pop();
        } else if (type === IARRAY) {
          argCount = item.value;
          args = [];
          while (argCount-- > 0) {
            args.unshift(nstack.pop());
          }
          nstack.push(args);
        } else {
          throw new Error('invalid Expression');
        }
      }
      if (nstack.length > 1) {
        throw new Error('invalid Expression (parity)');
      }
      // Explicitly return zero to avoid test issues caused by -0
      return nstack[0] === 0 ? 0 : resolveExpression(nstack[0], values);
    }

    function createExpressionEvaluator(token, expr, values) {
      if (isExpressionEvaluator(token)) return token;
      return {
        type: IEXPREVAL,
        value: function (scope) {
          return evaluate(token.value, expr, scope);
        }
      };
    }

    function isExpressionEvaluator(n) {
      return n && n.type === IEXPREVAL;
    }

    function resolveExpression(n, values) {
      return isExpressionEvaluator(n) ? n.value(values) : n;
    }

    function expressionToString(tokens, toJS) {
      var nstack = [];
      var n1, n2, n3;
      var f, args, argCount;
      for (var i = 0; i < tokens.length; i++) {
        var item = tokens[i];
        var type = item.type;
        if (type === INUMBER) {
          if (typeof item.value === 'number' && item.value < 0) {
            nstack.push('(' + item.value + ')');
          } else if (Array.isArray(item.value)) {
            nstack.push('[' + item.value.map(escapeValue).join(', ') + ']');
          } else {
            nstack.push(escapeValue(item.value));
          }
        } else if (type === IOP2) {
          n2 = nstack.pop();
          n1 = nstack.pop();
          f = item.value;
          if (toJS) {
            if (f === '^') {
              nstack.push('Math.pow(' + n1 + ', ' + n2 + ')');
            } else if (f === 'and') {
              nstack.push('(!!' + n1 + ' && !!' + n2 + ')');
            } else if (f === 'or') {
              nstack.push('(!!' + n1 + ' || !!' + n2 + ')');
            } else if (f === '||') {
              nstack.push('(function(a,b){ return Array.isArray(a) && Array.isArray(b) ? a.concat(b) : String(a) + String(b); }((' + n1 + '),(' + n2 + ')))');
            } else if (f === '==') {
              nstack.push('(' + n1 + ' === ' + n2 + ')');
            } else if (f === '!=') {
              nstack.push('(' + n1 + ' !== ' + n2 + ')');
            } else if (f === '[') {
              nstack.push(n1 + '[(' + n2 + ') | 0]');
            } else {
              nstack.push('(' + n1 + ' ' + f + ' ' + n2 + ')');
            }
          } else {
            if (f === '[') {
              nstack.push(n1 + '[' + n2 + ']');
            } else {
              nstack.push('(' + n1 + ' ' + f + ' ' + n2 + ')');
            }
          }
        } else if (type === IOP3) {
          n3 = nstack.pop();
          n2 = nstack.pop();
          n1 = nstack.pop();
          f = item.value;
          if (f === '?') {
            nstack.push('(' + n1 + ' ? ' + n2 + ' : ' + n3 + ')');
          } else {
            throw new Error('invalid Expression');
          }
        } else if (type === IVAR || type === IVARNAME) {
          nstack.push(item.value);
        } else if (type === IOP1) {
          n1 = nstack.pop();
          f = item.value;
          if (f === '-' || f === '+') {
            nstack.push('(' + f + n1 + ')');
          } else if (toJS) {
            if (f === 'not') {
              nstack.push('(' + '!' + n1 + ')');
            } else if (f === '!') {
              nstack.push('fac(' + n1 + ')');
            } else {
              nstack.push(f + '(' + n1 + ')');
            }
          } else if (f === '!') {
            nstack.push('(' + n1 + '!)');
          } else {
            nstack.push('(' + f + ' ' + n1 + ')');
          }
        } else if (type === IFUNCALL) {
          argCount = item.value;
          args = [];
          while (argCount-- > 0) {
            args.unshift(nstack.pop());
          }
          f = nstack.pop();
          nstack.push(f + '(' + args.join(', ') + ')');
        } else if (type === IFUNDEF) {
          n2 = nstack.pop();
          argCount = item.value;
          args = [];
          while (argCount-- > 0) {
            args.unshift(nstack.pop());
          }
          n1 = nstack.pop();
          if (toJS) {
            nstack.push('(' + n1 + ' = function(' + args.join(', ') + ') { return ' + n2 + ' })');
          } else {
            nstack.push('(' + n1 + '(' + args.join(', ') + ') = ' + n2 + ')');
          }
        } else if (type === IMEMBER) {
          n1 = nstack.pop();
          nstack.push(n1 + '.' + item.value);
        } else if (type === IARRAY) {
          argCount = item.value;
          args = [];
          while (argCount-- > 0) {
            args.unshift(nstack.pop());
          }
          nstack.push('[' + args.join(', ') + ']');
        } else if (type === IEXPR) {
          nstack.push('(' + expressionToString(item.value, toJS) + ')');
        } else if (type === IENDSTATEMENT) ; else {
          throw new Error('invalid Expression');
        }
      }
      if (nstack.length > 1) {
        if (toJS) {
          nstack = [ nstack.join(',') ];
        } else {
          nstack = [ nstack.join(';') ];
        }
      }
      return String(nstack[0]);
    }

    function escapeValue(v) {
      if (typeof v === 'string') {
        return JSON.stringify(v).replace(/\u2028/g, '\\u2028').replace(/\u2029/g, '\\u2029');
      }
      return v;
    }

    function contains(array, obj) {
      for (var i = 0; i < array.length; i++) {
        if (array[i] === obj) {
          return true;
        }
      }
      return false;
    }

    function getSymbols(tokens, symbols, options) {
      options = options || {};
      var withMembers = !!options.withMembers;
      var prevVar = null;

      for (var i = 0; i < tokens.length; i++) {
        var item = tokens[i];
        if (item.type === IVAR || item.type === IVARNAME) {
          if (!withMembers && !contains(symbols, item.value)) {
            symbols.push(item.value);
          } else if (prevVar !== null) {
            if (!contains(symbols, prevVar)) {
              symbols.push(prevVar);
            }
            prevVar = item.value;
          } else {
            prevVar = item.value;
          }
        } else if (item.type === IMEMBER && withMembers && prevVar !== null) {
          prevVar += '.' + item.value;
        } else if (item.type === IEXPR) {
          getSymbols(item.value, symbols, options);
        } else if (prevVar !== null) {
          if (!contains(symbols, prevVar)) {
            symbols.push(prevVar);
          }
          prevVar = null;
        }
      }

      if (prevVar !== null && !contains(symbols, prevVar)) {
        symbols.push(prevVar);
      }
    }

    function Expression(tokens, parser) {
      this.tokens = tokens;
      this.parser = parser;
      this.unaryOps = parser.unaryOps;
      this.binaryOps = parser.binaryOps;
      this.ternaryOps = parser.ternaryOps;
      this.functions = parser.functions;
    }

    Expression.prototype.simplify = function (values) {
      values = values || {};
      return new Expression(simplify(this.tokens, this.unaryOps, this.binaryOps, this.ternaryOps, values), this.parser);
    };

    Expression.prototype.substitute = function (variable, expr) {
      if (!(expr instanceof Expression)) {
        expr = this.parser.parse(String(expr));
      }

      return new Expression(substitute(this.tokens, variable, expr), this.parser);
    };

    Expression.prototype.evaluate = function (values) {
      values = values || {};
      return evaluate(this.tokens, this, values);
    };

    Expression.prototype.toString = function () {
      return expressionToString(this.tokens, false);
    };

    Expression.prototype.symbols = function (options) {
      options = options || {};
      var vars = [];
      getSymbols(this.tokens, vars, options);
      return vars;
    };

    Expression.prototype.variables = function (options) {
      options = options || {};
      var vars = [];
      getSymbols(this.tokens, vars, options);
      var functions = this.functions;
      return vars.filter(function (name) {
        return !(name in functions);
      });
    };

    Expression.prototype.toJSFunction = function (param, variables) {
      var expr = this;
      var f = new Function(param, 'with(this.functions) with (this.ternaryOps) with (this.binaryOps) with (this.unaryOps) { return ' + expressionToString(this.simplify(variables).tokens, true) + '; }'); // eslint-disable-line no-new-func
      return function () {
        return f.apply(expr, arguments);
      };
    };

    var TEOF = 'TEOF';
    var TOP = 'TOP';
    var TNUMBER = 'TNUMBER';
    var TSTRING = 'TSTRING';
    var TPAREN = 'TPAREN';
    var TBRACKET = 'TBRACKET';
    var TCOMMA = 'TCOMMA';
    var TNAME = 'TNAME';
    var TSEMICOLON = 'TSEMICOLON';

    function Token(type, value, index) {
      this.type = type;
      this.value = value;
      this.index = index;
    }

    Token.prototype.toString = function () {
      return this.type + ': ' + this.value;
    };

    function TokenStream(parser, expression) {
      this.pos = 0;
      this.current = null;
      this.unaryOps = parser.unaryOps;
      this.binaryOps = parser.binaryOps;
      this.ternaryOps = parser.ternaryOps;
      this.consts = parser.consts;
      this.expression = expression;
      this.savedPosition = 0;
      this.savedCurrent = null;
      this.options = parser.options;
      this.parser = parser;
    }

    TokenStream.prototype.newToken = function (type, value, pos) {
      return new Token(type, value, pos != null ? pos : this.pos);
    };

    TokenStream.prototype.save = function () {
      this.savedPosition = this.pos;
      this.savedCurrent = this.current;
    };

    TokenStream.prototype.restore = function () {
      this.pos = this.savedPosition;
      this.current = this.savedCurrent;
    };

    TokenStream.prototype.next = function () {
      if (this.pos >= this.expression.length) {
        return this.newToken(TEOF, 'EOF');
      }

      if (this.isWhitespace() || this.isComment()) {
        return this.next();
      } else if (this.isRadixInteger() ||
          this.isNumber() ||
          this.isOperator() ||
          this.isString() ||
          this.isParen() ||
          this.isBracket() ||
          this.isComma() ||
          this.isSemicolon() ||
          this.isNamedOp() ||
          this.isConst() ||
          this.isName()) {
        return this.current;
      } else {
        this.parseError('Unknown character "' + this.expression.charAt(this.pos) + '"');
      }
    };

    TokenStream.prototype.isString = function () {
      var r = false;
      var startPos = this.pos;
      var quote = this.expression.charAt(startPos);

      if (quote === '\'' || quote === '"') {
        var index = this.expression.indexOf(quote, startPos + 1);
        while (index >= 0 && this.pos < this.expression.length) {
          this.pos = index + 1;
          if (this.expression.charAt(index - 1) !== '\\') {
            var rawString = this.expression.substring(startPos + 1, index);
            this.current = this.newToken(TSTRING, this.unescape(rawString), startPos);
            r = true;
            break;
          }
          index = this.expression.indexOf(quote, index + 1);
        }
      }
      return r;
    };

    TokenStream.prototype.isParen = function () {
      var c = this.expression.charAt(this.pos);
      if (c === '(' || c === ')') {
        this.current = this.newToken(TPAREN, c);
        this.pos++;
        return true;
      }
      return false;
    };

    TokenStream.prototype.isBracket = function () {
      var c = this.expression.charAt(this.pos);
      if ((c === '[' || c === ']') && this.isOperatorEnabled('[')) {
        this.current = this.newToken(TBRACKET, c);
        this.pos++;
        return true;
      }
      return false;
    };

    TokenStream.prototype.isComma = function () {
      var c = this.expression.charAt(this.pos);
      if (c === ',') {
        this.current = this.newToken(TCOMMA, ',');
        this.pos++;
        return true;
      }
      return false;
    };

    TokenStream.prototype.isSemicolon = function () {
      var c = this.expression.charAt(this.pos);
      if (c === ';') {
        this.current = this.newToken(TSEMICOLON, ';');
        this.pos++;
        return true;
      }
      return false;
    };

    TokenStream.prototype.isConst = function () {
      var startPos = this.pos;
      var i = startPos;
      for (; i < this.expression.length; i++) {
        var c = this.expression.charAt(i);
        if (c.toUpperCase() === c.toLowerCase()) {
          if (i === this.pos || (c !== '_' && c !== '.' && (c < '0' || c > '9'))) {
            break;
          }
        }
      }
      if (i > startPos) {
        var str = this.expression.substring(startPos, i);
        if (str in this.consts) {
          this.current = this.newToken(TNUMBER, this.consts[str]);
          this.pos += str.length;
          return true;
        }
      }
      return false;
    };

    TokenStream.prototype.isNamedOp = function () {
      var startPos = this.pos;
      var i = startPos;
      for (; i < this.expression.length; i++) {
        var c = this.expression.charAt(i);
        if (c.toUpperCase() === c.toLowerCase()) {
          if (i === this.pos || (c !== '_' && (c < '0' || c > '9'))) {
            break;
          }
        }
      }
      if (i > startPos) {
        var str = this.expression.substring(startPos, i);
        if (this.isOperatorEnabled(str) && (str in this.binaryOps || str in this.unaryOps || str in this.ternaryOps)) {
          this.current = this.newToken(TOP, str);
          this.pos += str.length;
          return true;
        }
      }
      return false;
    };

    TokenStream.prototype.isName = function () {
      var startPos = this.pos;
      var i = startPos;
      var hasLetter = false;
      for (; i < this.expression.length; i++) {
        var c = this.expression.charAt(i);
        if (c.toUpperCase() === c.toLowerCase()) {
          if (i === this.pos && (c === '$' || c === '_')) {
            if (c === '_') {
              hasLetter = true;
            }
            continue;
          } else if (i === this.pos || !hasLetter || (c !== '_' && (c < '0' || c > '9'))) {
            break;
          }
        } else {
          hasLetter = true;
        }
      }
      if (hasLetter) {
        var str = this.expression.substring(startPos, i);
        this.current = this.newToken(TNAME, str);
        this.pos += str.length;
        return true;
      }
      return false;
    };

    TokenStream.prototype.isWhitespace = function () {
      var r = false;
      var c = this.expression.charAt(this.pos);
      while (c === ' ' || c === '\t' || c === '\n' || c === '\r') {
        r = true;
        this.pos++;
        if (this.pos >= this.expression.length) {
          break;
        }
        c = this.expression.charAt(this.pos);
      }
      return r;
    };

    var codePointPattern = /^[0-9a-f]{4}$/i;

    TokenStream.prototype.unescape = function (v) {
      var index = v.indexOf('\\');
      if (index < 0) {
        return v;
      }

      var buffer = v.substring(0, index);
      while (index >= 0) {
        var c = v.charAt(++index);
        switch (c) {
          case '\'':
            buffer += '\'';
            break;
          case '"':
            buffer += '"';
            break;
          case '\\':
            buffer += '\\';
            break;
          case '/':
            buffer += '/';
            break;
          case 'b':
            buffer += '\b';
            break;
          case 'f':
            buffer += '\f';
            break;
          case 'n':
            buffer += '\n';
            break;
          case 'r':
            buffer += '\r';
            break;
          case 't':
            buffer += '\t';
            break;
          case 'u':
            // interpret the following 4 characters as the hex of the unicode code point
            var codePoint = v.substring(index + 1, index + 5);
            if (!codePointPattern.test(codePoint)) {
              this.parseError('Illegal escape sequence: \\u' + codePoint);
            }
            buffer += String.fromCharCode(parseInt(codePoint, 16));
            index += 4;
            break;
          default:
            throw this.parseError('Illegal escape sequence: "\\' + c + '"');
        }
        ++index;
        var backslash = v.indexOf('\\', index);
        buffer += v.substring(index, backslash < 0 ? v.length : backslash);
        index = backslash;
      }

      return buffer;
    };

    TokenStream.prototype.isComment = function () {
      var c = this.expression.charAt(this.pos);
      if (c === '/' && this.expression.charAt(this.pos + 1) === '*') {
        this.pos = this.expression.indexOf('*/', this.pos) + 2;
        if (this.pos === 1) {
          this.pos = this.expression.length;
        }
        return true;
      }
      return false;
    };

    TokenStream.prototype.isRadixInteger = function () {
      var pos = this.pos;

      if (pos >= this.expression.length - 2 || this.expression.charAt(pos) !== '0') {
        return false;
      }
      ++pos;

      var radix;
      var validDigit;
      if (this.expression.charAt(pos) === 'x') {
        radix = 16;
        validDigit = /^[0-9a-f]$/i;
        ++pos;
      } else if (this.expression.charAt(pos) === 'b') {
        radix = 2;
        validDigit = /^[01]$/i;
        ++pos;
      } else {
        return false;
      }

      var valid = false;
      var startPos = pos;

      while (pos < this.expression.length) {
        var c = this.expression.charAt(pos);
        if (validDigit.test(c)) {
          pos++;
          valid = true;
        } else {
          break;
        }
      }

      if (valid) {
        this.current = this.newToken(TNUMBER, parseInt(this.expression.substring(startPos, pos), radix));
        this.pos = pos;
      }
      return valid;
    };

    TokenStream.prototype.isNumber = function () {
      var valid = false;
      var pos = this.pos;
      var startPos = pos;
      var resetPos = pos;
      var foundDot = false;
      var foundDigits = false;
      var c;

      while (pos < this.expression.length) {
        c = this.expression.charAt(pos);
        if ((c >= '0' && c <= '9') || (!foundDot && c === '.')) {
          if (c === '.') {
            foundDot = true;
          } else {
            foundDigits = true;
          }
          pos++;
          valid = foundDigits;
        } else {
          break;
        }
      }

      if (valid) {
        resetPos = pos;
      }

      if (c === 'e' || c === 'E') {
        pos++;
        var acceptSign = true;
        var validExponent = false;
        while (pos < this.expression.length) {
          c = this.expression.charAt(pos);
          if (acceptSign && (c === '+' || c === '-')) {
            acceptSign = false;
          } else if (c >= '0' && c <= '9') {
            validExponent = true;
            acceptSign = false;
          } else {
            break;
          }
          pos++;
        }

        if (!validExponent) {
          pos = resetPos;
        }
      }

      if (valid) {
        this.current = this.newToken(TNUMBER, parseFloat(this.expression.substring(startPos, pos)));
        this.pos = pos;
      } else {
        this.pos = resetPos;
      }
      return valid;
    };

    TokenStream.prototype.isOperator = function () {
      var startPos = this.pos;
      var c = this.expression.charAt(this.pos);

      if (c === '+' || c === '-' || c === '*' || c === '/' || c === '%' || c === '^' || c === '?' || c === ':' || c === '.') {
        this.current = this.newToken(TOP, c);
      } else if (c === '∙' || c === '•') {
        this.current = this.newToken(TOP, '*');
      } else if (c === '>') {
        if (this.expression.charAt(this.pos + 1) === '=') {
          this.current = this.newToken(TOP, '>=');
          this.pos++;
        } else {
          this.current = this.newToken(TOP, '>');
        }
      } else if (c === '<') {
        if (this.expression.charAt(this.pos + 1) === '=') {
          this.current = this.newToken(TOP, '<=');
          this.pos++;
        } else {
          this.current = this.newToken(TOP, '<');
        }
      } else if (c === '|') {
        if (this.expression.charAt(this.pos + 1) === '|') {
          this.current = this.newToken(TOP, '||');
          this.pos++;
        } else {
          return false;
        }
      } else if (c === '=') {
        if (this.expression.charAt(this.pos + 1) === '=') {
          this.current = this.newToken(TOP, '==');
          this.pos++;
        } else {
          this.current = this.newToken(TOP, c);
        }
      } else if (c === '!') {
        if (this.expression.charAt(this.pos + 1) === '=') {
          this.current = this.newToken(TOP, '!=');
          this.pos++;
        } else {
          this.current = this.newToken(TOP, c);
        }
      } else {
        return false;
      }
      this.pos++;

      if (this.isOperatorEnabled(this.current.value)) {
        return true;
      } else {
        this.pos = startPos;
        return false;
      }
    };

    TokenStream.prototype.isOperatorEnabled = function (op) {
      return this.parser.isOperatorEnabled(op);
    };

    TokenStream.prototype.getCoordinates = function () {
      var line = 0;
      var column;
      var newline = -1;
      do {
        line++;
        column = this.pos - newline;
        newline = this.expression.indexOf('\n', newline + 1);
      } while (newline >= 0 && newline < this.pos);

      return {
        line: line,
        column: column
      };
    };

    TokenStream.prototype.parseError = function (msg) {
      var coords = this.getCoordinates();
      throw new Error('parse error [' + coords.line + ':' + coords.column + ']: ' + msg);
    };

    function ParserState(parser, tokenStream, options) {
      this.parser = parser;
      this.tokens = tokenStream;
      this.current = null;
      this.nextToken = null;
      this.next();
      this.savedCurrent = null;
      this.savedNextToken = null;
      this.allowMemberAccess = options.allowMemberAccess !== false;
    }

    ParserState.prototype.next = function () {
      this.current = this.nextToken;
      return (this.nextToken = this.tokens.next());
    };

    ParserState.prototype.tokenMatches = function (token, value) {
      if (typeof value === 'undefined') {
        return true;
      } else if (Array.isArray(value)) {
        return contains(value, token.value);
      } else if (typeof value === 'function') {
        return value(token);
      } else {
        return token.value === value;
      }
    };

    ParserState.prototype.save = function () {
      this.savedCurrent = this.current;
      this.savedNextToken = this.nextToken;
      this.tokens.save();
    };

    ParserState.prototype.restore = function () {
      this.tokens.restore();
      this.current = this.savedCurrent;
      this.nextToken = this.savedNextToken;
    };

    ParserState.prototype.accept = function (type, value) {
      if (this.nextToken.type === type && this.tokenMatches(this.nextToken, value)) {
        this.next();
        return true;
      }
      return false;
    };

    ParserState.prototype.expect = function (type, value) {
      if (!this.accept(type, value)) {
        var coords = this.tokens.getCoordinates();
        throw new Error('parse error [' + coords.line + ':' + coords.column + ']: Expected ' + (value || type));
      }
    };

    ParserState.prototype.parseAtom = function (instr) {
      var unaryOps = this.tokens.unaryOps;
      function isPrefixOperator(token) {
        return token.value in unaryOps;
      }

      if (this.accept(TNAME) || this.accept(TOP, isPrefixOperator)) {
        instr.push(new Instruction(IVAR, this.current.value));
      } else if (this.accept(TNUMBER)) {
        instr.push(new Instruction(INUMBER, this.current.value));
      } else if (this.accept(TSTRING)) {
        instr.push(new Instruction(INUMBER, this.current.value));
      } else if (this.accept(TPAREN, '(')) {
        this.parseExpression(instr);
        this.expect(TPAREN, ')');
      } else if (this.accept(TBRACKET, '[')) {
        if (this.accept(TBRACKET, ']')) {
          instr.push(new Instruction(IARRAY, 0));
        } else {
          var argCount = this.parseArrayList(instr);
          instr.push(new Instruction(IARRAY, argCount));
        }
      } else {
        throw new Error('unexpected ' + this.nextToken);
      }
    };

    ParserState.prototype.parseExpression = function (instr) {
      var exprInstr = [];
      if (this.parseUntilEndStatement(instr, exprInstr)) {
        return;
      }
      this.parseVariableAssignmentExpression(exprInstr);
      if (this.parseUntilEndStatement(instr, exprInstr)) {
        return;
      }
      this.pushExpression(instr, exprInstr);
    };

    ParserState.prototype.pushExpression = function (instr, exprInstr) {
      for (var i = 0, len = exprInstr.length; i < len; i++) {
        instr.push(exprInstr[i]);
      }
    };

    ParserState.prototype.parseUntilEndStatement = function (instr, exprInstr) {
      if (!this.accept(TSEMICOLON)) return false;
      if (this.nextToken && this.nextToken.type !== TEOF && !(this.nextToken.type === TPAREN && this.nextToken.value === ')')) {
        exprInstr.push(new Instruction(IENDSTATEMENT));
      }
      if (this.nextToken.type !== TEOF) {
        this.parseExpression(exprInstr);
      }
      instr.push(new Instruction(IEXPR, exprInstr));
      return true;
    };

    ParserState.prototype.parseArrayList = function (instr) {
      var argCount = 0;

      while (!this.accept(TBRACKET, ']')) {
        this.parseExpression(instr);
        ++argCount;
        while (this.accept(TCOMMA)) {
          this.parseExpression(instr);
          ++argCount;
        }
      }

      return argCount;
    };

    ParserState.prototype.parseVariableAssignmentExpression = function (instr) {
      this.parseConditionalExpression(instr);
      while (this.accept(TOP, '=')) {
        var varName = instr.pop();
        var varValue = [];
        var lastInstrIndex = instr.length - 1;
        if (varName.type === IFUNCALL) {
          if (!this.tokens.isOperatorEnabled('()=')) {
            throw new Error('function definition is not permitted');
          }
          for (var i = 0, len = varName.value + 1; i < len; i++) {
            var index = lastInstrIndex - i;
            if (instr[index].type === IVAR) {
              instr[index] = new Instruction(IVARNAME, instr[index].value);
            }
          }
          this.parseVariableAssignmentExpression(varValue);
          instr.push(new Instruction(IEXPR, varValue));
          instr.push(new Instruction(IFUNDEF, varName.value));
          continue;
        }
        if (varName.type !== IVAR && varName.type !== IMEMBER) {
          throw new Error('expected variable for assignment');
        }
        this.parseVariableAssignmentExpression(varValue);
        instr.push(new Instruction(IVARNAME, varName.value));
        instr.push(new Instruction(IEXPR, varValue));
        instr.push(binaryInstruction('='));
      }
    };

    ParserState.prototype.parseConditionalExpression = function (instr) {
      this.parseOrExpression(instr);
      while (this.accept(TOP, '?')) {
        var trueBranch = [];
        var falseBranch = [];
        this.parseConditionalExpression(trueBranch);
        this.expect(TOP, ':');
        this.parseConditionalExpression(falseBranch);
        instr.push(new Instruction(IEXPR, trueBranch));
        instr.push(new Instruction(IEXPR, falseBranch));
        instr.push(ternaryInstruction('?'));
      }
    };

    ParserState.prototype.parseOrExpression = function (instr) {
      this.parseAndExpression(instr);
      while (this.accept(TOP, 'or')) {
        var falseBranch = [];
        this.parseAndExpression(falseBranch);
        instr.push(new Instruction(IEXPR, falseBranch));
        instr.push(binaryInstruction('or'));
      }
    };

    ParserState.prototype.parseAndExpression = function (instr) {
      this.parseComparison(instr);
      while (this.accept(TOP, 'and')) {
        var trueBranch = [];
        this.parseComparison(trueBranch);
        instr.push(new Instruction(IEXPR, trueBranch));
        instr.push(binaryInstruction('and'));
      }
    };

    var COMPARISON_OPERATORS = ['==', '!=', '<', '<=', '>=', '>', 'in'];

    ParserState.prototype.parseComparison = function (instr) {
      this.parseAddSub(instr);
      while (this.accept(TOP, COMPARISON_OPERATORS)) {
        var op = this.current;
        this.parseAddSub(instr);
        instr.push(binaryInstruction(op.value));
      }
    };

    var ADD_SUB_OPERATORS = ['+', '-', '||'];

    ParserState.prototype.parseAddSub = function (instr) {
      this.parseTerm(instr);
      while (this.accept(TOP, ADD_SUB_OPERATORS)) {
        var op = this.current;
        this.parseTerm(instr);
        instr.push(binaryInstruction(op.value));
      }
    };

    var TERM_OPERATORS = ['*', '/', '%'];

    ParserState.prototype.parseTerm = function (instr) {
      this.parseFactor(instr);
      while (this.accept(TOP, TERM_OPERATORS)) {
        var op = this.current;
        this.parseFactor(instr);
        instr.push(binaryInstruction(op.value));
      }
    };

    ParserState.prototype.parseFactor = function (instr) {
      var unaryOps = this.tokens.unaryOps;
      function isPrefixOperator(token) {
        return token.value in unaryOps;
      }

      this.save();
      if (this.accept(TOP, isPrefixOperator)) {
        if (this.current.value !== '-' && this.current.value !== '+') {
          if (this.nextToken.type === TPAREN && this.nextToken.value === '(') {
            this.restore();
            this.parseExponential(instr);
            return;
          } else if (this.nextToken.type === TSEMICOLON || this.nextToken.type === TCOMMA || this.nextToken.type === TEOF || (this.nextToken.type === TPAREN && this.nextToken.value === ')')) {
            this.restore();
            this.parseAtom(instr);
            return;
          }
        }

        var op = this.current;
        this.parseFactor(instr);
        instr.push(unaryInstruction(op.value));
      } else {
        this.parseExponential(instr);
      }
    };

    ParserState.prototype.parseExponential = function (instr) {
      this.parsePostfixExpression(instr);
      while (this.accept(TOP, '^')) {
        this.parseFactor(instr);
        instr.push(binaryInstruction('^'));
      }
    };

    ParserState.prototype.parsePostfixExpression = function (instr) {
      this.parseFunctionCall(instr);
      while (this.accept(TOP, '!')) {
        instr.push(unaryInstruction('!'));
      }
    };

    ParserState.prototype.parseFunctionCall = function (instr) {
      var unaryOps = this.tokens.unaryOps;
      function isPrefixOperator(token) {
        return token.value in unaryOps;
      }

      if (this.accept(TOP, isPrefixOperator)) {
        var op = this.current;
        this.parseAtom(instr);
        instr.push(unaryInstruction(op.value));
      } else {
        this.parseMemberExpression(instr);
        while (this.accept(TPAREN, '(')) {
          if (this.accept(TPAREN, ')')) {
            instr.push(new Instruction(IFUNCALL, 0));
          } else {
            var argCount = this.parseArgumentList(instr);
            instr.push(new Instruction(IFUNCALL, argCount));
          }
        }
      }
    };

    ParserState.prototype.parseArgumentList = function (instr) {
      var argCount = 0;

      while (!this.accept(TPAREN, ')')) {
        this.parseExpression(instr);
        ++argCount;
        while (this.accept(TCOMMA)) {
          this.parseExpression(instr);
          ++argCount;
        }
      }

      return argCount;
    };

    ParserState.prototype.parseMemberExpression = function (instr) {
      this.parseAtom(instr);
      while (this.accept(TOP, '.') || this.accept(TBRACKET, '[')) {
        var op = this.current;

        if (op.value === '.') {
          if (!this.allowMemberAccess) {
            throw new Error('unexpected ".", member access is not permitted');
          }

          this.expect(TNAME);
          instr.push(new Instruction(IMEMBER, this.current.value));
        } else if (op.value === '[') {
          if (!this.tokens.isOperatorEnabled('[')) {
            throw new Error('unexpected "[]", arrays are disabled');
          }

          this.parseExpression(instr);
          this.expect(TBRACKET, ']');
          instr.push(binaryInstruction('['));
        } else {
          throw new Error('unexpected symbol: ' + op.value);
        }
      }
    };

    function add(a, b) {
      return Number(a) + Number(b);
    }

    function sub(a, b) {
      return a - b;
    }

    function mul(a, b) {
      return a * b;
    }

    function div(a, b) {
      return a / b;
    }

    function mod(a, b) {
      return a % b;
    }

    function concat(a, b) {
      if (Array.isArray(a) && Array.isArray(b)) {
        return a.concat(b);
      }
      return '' + a + b;
    }

    function equal(a, b) {
      return a === b;
    }

    function notEqual(a, b) {
      return a !== b;
    }

    function greaterThan(a, b) {
      return a > b;
    }

    function lessThan(a, b) {
      return a < b;
    }

    function greaterThanEqual(a, b) {
      return a >= b;
    }

    function lessThanEqual(a, b) {
      return a <= b;
    }

    function andOperator(a, b) {
      return Boolean(a && b);
    }

    function orOperator(a, b) {
      return Boolean(a || b);
    }

    function inOperator(a, b) {
      return contains(b, a);
    }

    function sinh(a) {
      return ((Math.exp(a) - Math.exp(-a)) / 2);
    }

    function cosh(a) {
      return ((Math.exp(a) + Math.exp(-a)) / 2);
    }

    function tanh(a) {
      if (a === Infinity) return 1;
      if (a === -Infinity) return -1;
      return (Math.exp(a) - Math.exp(-a)) / (Math.exp(a) + Math.exp(-a));
    }

    function asinh(a) {
      if (a === -Infinity) return a;
      return Math.log(a + Math.sqrt((a * a) + 1));
    }

    function acosh(a) {
      return Math.log(a + Math.sqrt((a * a) - 1));
    }

    function atanh(a) {
      return (Math.log((1 + a) / (1 - a)) / 2);
    }

    function log10(a) {
      return Math.log(a) * Math.LOG10E;
    }

    function neg(a) {
      return -a;
    }

    function not(a) {
      return !a;
    }

    function trunc(a) {
      return a < 0 ? Math.ceil(a) : Math.floor(a);
    }

    function random(a) {
      return Math.random() * (a || 1);
    }

    function factorial(a) { // a!
      return gamma(a + 1);
    }

    function isInteger(value) {
      return isFinite(value) && (value === Math.round(value));
    }

    var GAMMA_G = 4.7421875;
    var GAMMA_P = [
      0.99999999999999709182,
      57.156235665862923517, -59.597960355475491248,
      14.136097974741747174, -0.49191381609762019978,
      0.33994649984811888699e-4,
      0.46523628927048575665e-4, -0.98374475304879564677e-4,
      0.15808870322491248884e-3, -0.21026444172410488319e-3,
      0.21743961811521264320e-3, -0.16431810653676389022e-3,
      0.84418223983852743293e-4, -0.26190838401581408670e-4,
      0.36899182659531622704e-5
    ];

    // Gamma function from math.js
    function gamma(n) {
      var t, x;

      if (isInteger(n)) {
        if (n <= 0) {
          return isFinite(n) ? Infinity : NaN;
        }

        if (n > 171) {
          return Infinity; // Will overflow
        }

        var value = n - 2;
        var res = n - 1;
        while (value > 1) {
          res *= value;
          value--;
        }

        if (res === 0) {
          res = 1; // 0! is per definition 1
        }

        return res;
      }

      if (n < 0.5) {
        return Math.PI / (Math.sin(Math.PI * n) * gamma(1 - n));
      }

      if (n >= 171.35) {
        return Infinity; // will overflow
      }

      if (n > 85.0) { // Extended Stirling Approx
        var twoN = n * n;
        var threeN = twoN * n;
        var fourN = threeN * n;
        var fiveN = fourN * n;
        return Math.sqrt(2 * Math.PI / n) * Math.pow((n / Math.E), n) *
          (1 + (1 / (12 * n)) + (1 / (288 * twoN)) - (139 / (51840 * threeN)) -
          (571 / (2488320 * fourN)) + (163879 / (209018880 * fiveN)) +
          (5246819 / (75246796800 * fiveN * n)));
      }

      --n;
      x = GAMMA_P[0];
      for (var i = 1; i < GAMMA_P.length; ++i) {
        x += GAMMA_P[i] / (n + i);
      }

      t = n + GAMMA_G + 0.5;
      return Math.sqrt(2 * Math.PI) * Math.pow(t, n + 0.5) * Math.exp(-t) * x;
    }

    function stringOrArrayLength(s) {
      if (Array.isArray(s)) {
        return s.length;
      }
      return String(s).length;
    }

    function hypot() {
      var sum = 0;
      var larg = 0;
      for (var i = 0; i < arguments.length; i++) {
        var arg = Math.abs(arguments[i]);
        var div;
        if (larg < arg) {
          div = larg / arg;
          sum = (sum * div * div) + 1;
          larg = arg;
        } else if (arg > 0) {
          div = arg / larg;
          sum += div * div;
        } else {
          sum += arg;
        }
      }
      return larg === Infinity ? Infinity : larg * Math.sqrt(sum);
    }

    function condition(cond, yep, nope) {
      return cond ? yep : nope;
    }

    /**
    * Decimal adjustment of a number.
    * From @escopecz.
    *
    * @param {Number} value The number.
    * @param {Integer} exp  The exponent (the 10 logarithm of the adjustment base).
    * @return {Number} The adjusted value.
    */
    function roundTo(value, exp) {
      // If the exp is undefined or zero...
      if (typeof exp === 'undefined' || +exp === 0) {
        return Math.round(value);
      }
      value = +value;
      exp = -(+exp);
      // If the value is not a number or the exp is not an integer...
      if (isNaN(value) || !(typeof exp === 'number' && exp % 1 === 0)) {
        return NaN;
      }
      // Shift
      value = value.toString().split('e');
      value = Math.round(+(value[0] + 'e' + (value[1] ? (+value[1] - exp) : -exp)));
      // Shift back
      value = value.toString().split('e');
      return +(value[0] + 'e' + (value[1] ? (+value[1] + exp) : exp));
    }

    function setVar(name, value, variables) {
      if (variables) variables[name] = value;
      return value;
    }

    function arrayIndex(array, index) {
      return array[index | 0];
    }

    function max(array) {
      if (arguments.length === 1 && Array.isArray(array)) {
        return Math.max.apply(Math, array);
      } else {
        return Math.max.apply(Math, arguments);
      }
    }

    function min(array) {
      if (arguments.length === 1 && Array.isArray(array)) {
        return Math.min.apply(Math, array);
      } else {
        return Math.min.apply(Math, arguments);
      }
    }

    function arrayMap(f, a) {
      if (typeof f !== 'function') {
        throw new Error('First argument to map is not a function');
      }
      if (!Array.isArray(a)) {
        throw new Error('Second argument to map is not an array');
      }
      return a.map(function (x, i) {
        return f(x, i);
      });
    }

    function arrayFold(f, init, a) {
      if (typeof f !== 'function') {
        throw new Error('First argument to fold is not a function');
      }
      if (!Array.isArray(a)) {
        throw new Error('Second argument to fold is not an array');
      }
      return a.reduce(function (acc, x, i) {
        return f(acc, x, i);
      }, init);
    }

    function arrayFilter(f, a) {
      if (typeof f !== 'function') {
        throw new Error('First argument to filter is not a function');
      }
      if (!Array.isArray(a)) {
        throw new Error('Second argument to filter is not an array');
      }
      return a.filter(function (x, i) {
        return f(x, i);
      });
    }

    function stringOrArrayIndexOf(target, s) {
      if (!(Array.isArray(s) || typeof s === 'string')) {
        throw new Error('Second argument to indexOf is not a string or array');
      }

      return s.indexOf(target);
    }

    function arrayJoin(sep, a) {
      if (!Array.isArray(a)) {
        throw new Error('Second argument to join is not an array');
      }

      return a.join(sep);
    }

    function sign(x) {
      return ((x > 0) - (x < 0)) || +x;
    }

    var ONE_THIRD = 1/3;
    function cbrt(x) {
      return x < 0 ? -Math.pow(-x, ONE_THIRD) : Math.pow(x, ONE_THIRD);
    }

    function expm1(x) {
      return Math.exp(x) - 1;
    }

    function log1p(x) {
      return Math.log(1 + x);
    }

    function log2(x) {
      return Math.log(x) / Math.LN2;
    }

    function Parser(options) {
      this.options = options || {};
      this.unaryOps = {
        sin: Math.sin,
        cos: Math.cos,
        tan: Math.tan,
        asin: Math.asin,
        acos: Math.acos,
        atan: Math.atan,
        sinh: Math.sinh || sinh,
        cosh: Math.cosh || cosh,
        tanh: Math.tanh || tanh,
        asinh: Math.asinh || asinh,
        acosh: Math.acosh || acosh,
        atanh: Math.atanh || atanh,
        sqrt: Math.sqrt,
        cbrt: Math.cbrt || cbrt,
        log: Math.log,
        log2: Math.log2 || log2,
        ln: Math.log,
        lg: Math.log10 || log10,
        log10: Math.log10 || log10,
        expm1: Math.expm1 || expm1,
        log1p: Math.log1p || log1p,
        abs: Math.abs,
        ceil: Math.ceil,
        floor: Math.floor,
        round: Math.round,
        trunc: Math.trunc || trunc,
        '-': neg,
        '+': Number,
        exp: Math.exp,
        not: not,
        length: stringOrArrayLength,
        '!': factorial,
        sign: Math.sign || sign
      };

      this.binaryOps = {
        '+': add,
        '-': sub,
        '*': mul,
        '/': div,
        '%': mod,
        '^': Math.pow,
        '||': concat,
        '==': equal,
        '!=': notEqual,
        '>': greaterThan,
        '<': lessThan,
        '>=': greaterThanEqual,
        '<=': lessThanEqual,
        and: andOperator,
        or: orOperator,
        'in': inOperator,
        '=': setVar,
        '[': arrayIndex
      };

      this.ternaryOps = {
        '?': condition
      };

      this.functions = {
        random: random,
        fac: factorial,
        min: min,
        max: max,
        hypot: Math.hypot || hypot,
        pyt: Math.hypot || hypot, // backward compat
        pow: Math.pow,
        atan2: Math.atan2,
        'if': condition,
        gamma: gamma,
        roundTo: roundTo,
        map: arrayMap,
        fold: arrayFold,
        filter: arrayFilter,
        indexOf: stringOrArrayIndexOf,
        join: arrayJoin
      };

      this.consts = {
        E: Math.E,
        PI: Math.PI,
        'true': true,
        'false': false
      };
    }

    Parser.prototype.parse = function (expr) {
      var instr = [];
      var parserState = new ParserState(
        this,
        new TokenStream(this, expr),
        { allowMemberAccess: this.options.allowMemberAccess }
      );

      parserState.parseExpression(instr);
      parserState.expect(TEOF, 'EOF');

      return new Expression(instr, this);
    };

    Parser.prototype.evaluate = function (expr, variables) {
      return this.parse(expr).evaluate(variables);
    };

    var sharedParser = new Parser();

    Parser.parse = function (expr) {
      return sharedParser.parse(expr);
    };

    Parser.evaluate = function (expr, variables) {
      return sharedParser.parse(expr).evaluate(variables);
    };

    var optionNameMap = {
      '+': 'add',
      '-': 'subtract',
      '*': 'multiply',
      '/': 'divide',
      '%': 'remainder',
      '^': 'power',
      '!': 'factorial',
      '<': 'comparison',
      '>': 'comparison',
      '<=': 'comparison',
      '>=': 'comparison',
      '==': 'comparison',
      '!=': 'comparison',
      '||': 'concatenate',
      'and': 'logical',
      'or': 'logical',
      'not': 'logical',
      '?': 'conditional',
      ':': 'conditional',
      '=': 'assignment',
      '[': 'array',
      '()=': 'fndef'
    };

    function getOptionName(op) {
      return optionNameMap.hasOwnProperty(op) ? optionNameMap[op] : op;
    }

    Parser.prototype.isOperatorEnabled = function (op) {
      var optionName = getOptionName(op);
      var operators = this.options.operators || {};

      return !(optionName in operators) || !!operators[optionName];
    };

    /*!
     Based on ndef.parser, by Raphael Graf(r@undefined.ch)
     http://www.undefined.ch/mparser/index.html

     Ported to JavaScript and modified by Matthew Crumley (email@matthewcrumley.com, http://silentmatt.com/)

     You are free to use and modify this code in anyway you find useful. Please leave this comment in the code
     to acknowledge its original source. If you feel like it, I enjoy hearing about projects that use my code,
     but don't feel like you have to let me know or ask permission.
    */

    // Backwards compatibility
    var index = {
      Parser: Parser,
      Expression: Expression
    };

    const parser = new Parser();

    parser.unaryOps = {
        '-': parser.unaryOps['-'],
        '+': parser.unaryOps['+'],
        '!': parser.unaryOps['!'],
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
    };

    parser.consts = {
        ...parser.consts,
        pi: Math.PI,
        e: Math.E,
    };

    function evaluateExpression(expr) {
        return parser.evaluate(expr);
    }

    class XacroParser {

        constructor() {
            this.inOrder = true;
            this.requirePrefix = true;
            this.localProperties = true;
            this.rospackCommands = {};
            this.workingPath = '';
        }

        async getFileContents(path) {
            throw new Error('XacroParser: getFileContents() not implemented.');
        }

        async parse(data) {

            /* Evaluation */
            // Evaluate expressions and rospack commands in attribute text
            // TODO: expressions can basically be any python expression
            function evaluateAttribute(str, properties, finalValue = false) {

                // recursively unpack parameters
                function unpackParams(str, properties) {

                    // if we're unpacking something that's already a number then just return
                    if (typeof str === 'number') {
                        return str;
                    }

                    // process all of the ${} and $() expressions
                    const res = str.replace(/(\$?\$\([^)]+\))|(\$?\${[^}]+})/g, match => {

                        // if we encounter an escaped $$ then return early
                        if (/^\$\$/.test(match)) {
                            return match;
                        }

                        const isRospackCommand = /^\$\(/.test(match);
                        let contents = match.substring(2, match.length - 1);
                        contents = unpackParams(contents, properties);

                        if (isRospackCommand) {

                            const command = unpackParams(contents, properties);
                            const tokens = command.split(/\s+/g);
                            const stem = tokens.shift();

                            try {
                                return handleRospackCommand(stem, ...tokens);
                            } catch (e) {
                                throw new Error(`XacroParser: Cannot run rospack command "${ contents }".\n` + e.message);
                            }

                        } else {
                            if (stack.includes(contents)) {
                                throw new Error(
                                    `XacroParser: Cannot evaluate infinitely recursive expression: ${
                                    stack.join(' > ')
                                } > ${
                                    contents
                                }`
                                );
                            }

                            stack.push(contents);

                            const operators = /(()|()|()|[()/*+\-%|&=[\]])+/g;
                            const expr = tokenize(contents)
                                .map(t => {
                                    operators.lastIndex = 0;
                                    if (isOperator(t)) return t;
                                    if (isNumber(t)) return t;
                                    if (isString(t)) return t;

                                    if (t in properties) {
                                        const arg = unpackParams(properties[t], properties);
                                        if (!isNumber(arg)) {
                                            return `"${ arg.toString().replace(/\\/g, '\\\\').replace(/"/g, '\\"') }"`;
                                        } else {
                                            return arg;
                                        }
                                    } else {
                                        return t;
                                    }
                                })
                                .join('');

                            stack.pop();

                            if (isString(expr)) {
                                return expr.substring(1, expr.length - 1);
                            } else if (isNumber(expr)) {
                                return expr;
                            } else {
                                const cleanExpr = normalizeExpression(expr);
                                return handleExpressionEvaluation(cleanExpr);
                            }
                        }
                    });

                    return res;

                }

                const stack = [];
                const allProps = mergePropertySets(globalProperties, properties);
                try {
                    // fix the escaped dollar signs only at the end to prevent double evaluation and only
                    // if the value is not an intermediate value like a computed property.
                    let result = unpackParams(str, allProps);
                    if (finalValue) {
                        result = result.replace(/\${2}([({])/g, (val, brace) => `$${ brace }`);
                    }
                    return result;
                } catch (e) {
                    throw new Error(`XacroParser: Failed to process expression "${ str }". \n` + e.message);
                }

            }

            // Evaluate the given node as a macro
            async function evaluateMacro(node, properties, macros, resultsList) {

                // Find the macro
                const macroName = node.tagName.replace(/^xacro:/, '');
                const macro = macros[macroName];

                if (!macro) {
                    throw new Error(`XacroParser: Cannot find macro "${ macroName }"`);
                }

                // Copy the properties and macros so we can modify them with
                // macro input fields and local macro definitions.
                const ogProperties = properties;
                const ogMacros = macros;
                properties = createNewPropertyScope(properties);
                macros = mergePropertySets(macros);

                // Modify the properties with macro param inputs
                let children = [];
                for (const c of node.children) {
                    await processNode(c, ogProperties, ogMacros, children);
                }
                children = children.filter(c => c.nodeType === c.ELEMENT_NODE);

                let blockCount = 0;
                for (const p in macro.params) {
                    const param = macro.params[p];
                    if (node.hasAttribute(p)) {
                        properties[p] = evaluateAttribute(node.getAttribute(p), ogProperties);
                    } else if (param.type === 'BLOCK') {
                        properties[p] = [children[blockCount]];
                        blockCount++;
                    } else if (param.type === 'MULTI_BLOCK') {
                        properties[p] = [...children.filter(c => c.tagName === p)[0].childNodes];
                    } else {
                        properties[p] = evaluateAttribute(macro.params[p].def, ogProperties);
                    }
                }

                // Expand the macro
                const macroChildren = [...macro.node.childNodes];
                for (const c of macroChildren) {
                    const nodes = [];
                    await processNode(c, properties, macros, nodes);
                    resultsList.push(...nodes);
                }
            }

            /* Parsing */
            // Conver the params into an object representation
            function parseMacroParam(param) {
                const obj = {};

                // Save the type of parameter
                // - two asterisks means an element expands input multiple
                // - one asterisk means copy the first elemnt
                // - no asterisks means value param
                if (/^\*\*/.test(param)) {
                    obj.type = 'MULTI_BLOCK';
                } else if (/^\*/.test(param)) {
                    obj.type = 'BLOCK';
                } else {
                    obj.type = 'PARAM';
                }

                // strip the asterisks
                param = param.replace(/^\*{1,2}/g, '');

                // Check if a default value is provided
                if (/:=/.test(param)) {
                    const [name, def] = param.split(':=');

                    // TODO: Support caret and default syntax
                    // TODO: is there any difference between the := and = syntax?
                    if (/^\^/.test(def) || /\|/.test(def)) {
                        throw new Error(`XacroParser: ROS Jade pass-through notation not supported in macro defaults: ${ def }`);
                    }

                    obj.name = name;
                    obj.def = def;
                } else {
                    obj.name = param;
                    obj.def = null;
                }

                return obj;
            }

            // Parse a xacro:macro tag
            function parseMacro(node) {
                // get attributes
                const name = node.getAttribute('name').replace(/^xacro:/, '');
                const params = node.getAttribute('params');

                // parse params
                const inputMap = {};
                if (params) {
                    const inputs = params
                        .trim()
                        .split(/\s+/g)
                        .map(s => parseMacroParam(s));
                    inputs.forEach(inp => {
                        inputMap[inp.name] = inp;
                    });
                }

                return {
                    name,
                    node: deepClone(node, false),
                    params: inputMap,
                };
            }

            // Recursively process and expand a node
            async function processNode(node, properties, macros, resultsList = []) {
                if (node.nodeType !== node.ELEMENT_NODE) {
                    const res = node.cloneNode();
                    res.textContent = evaluateAttribute(res.textContent, properties, true);
                    resultsList.push(res);
                    return;
                }

                let tagName = node.tagName.toLowerCase();
                if (!requirePrefix) {
                    switch (tagName) {

                        case 'arg':
                        case 'property':
                        case 'macro':
                        case 'insert_block':
                        case 'if':
                        case 'unless':
                        case 'include':
                        case 'element':
                        case 'attribute':
                            tagName = `xacro:${ tagName }`;
                            break;
                        default:
                            if (tagName in macros) {
                                tagName = `xacro:${ tagName }`;
                            }
                            break;

                    }
                }

                switch (tagName) {

                    case 'xacro:property': {
                        removeEndCommentsFromArray(resultsList);

                        const name = node.getAttribute('name');

                        let value;
                        if (node.hasAttribute('value')) {
                            value = node.getAttribute('value');
                        } else if (node.hasAttribute('default')) {
                            value = node.getAttribute('default');
                        } else {
                            const childNodes = [...node.childNodes];
                            value = [];
                            for (const c of childNodes) {
                                value.push(deepClone(c, false));
                            }
                        }

                        let scope = 'global';
                        if (localProperties) {
                            scope = node.getAttribute('scope') || 'local';
                        }

                        // Emulated behavior here
                        // https://github.com/ros/xacro/blob/melodic-devel/src/xacro/__init__.py#L565
                        if (scope !== 'local') {
                            value = evaluateAttribute(value, properties);
                        }

                        if (scope === 'global') {
                            globalProperties[name] = value;
                        } else if (scope === 'parent') {
                            properties[PARENT_SCOPE][name] = value;
                        } else {
                            properties[name] = value;
                        }

                        break;
                    }
                    case 'xacro:macro': {
                        removeEndCommentsFromArray(resultsList);

                        const macro = parseMacro(node);
                        macros[macro.name] = macro;
                        break;
                    }
                    case 'xacro:insert_block': {
                        removeEndCommentsFromArray(resultsList);

                        const name = node.getAttribute('name');
                        const nodes = properties[name];

                        for (const c of nodes) {
                            await processNode(c, properties, macros, resultsList);
                        }
                        return;
                    }
                    case 'xacro:if':
                    case 'xacro:unless': {
                        removeEndCommentsFromArray(resultsList);

                        const value = evaluateAttribute(node.getAttribute('value'), properties, true);
                        let bool = null;
                        if (!isNaN(parseFloat(value))) {
                            bool = !!parseFloat(value);
                        } else if (value === 'true' || value === 'false') {
                            bool = value === 'true';
                        } else {
                            bool = value;
                        }

                        if (tagName === 'xacro:unless') {
                            bool = !bool;
                        }

                        if (bool) {
                            const childNodes = [...node.childNodes];
                            for (const c of childNodes) {
                                await processNode(c, properties, macros, resultsList);
                            }
                        }
                        return;
                    }
                    case 'xacro:include': {
                        removeEndCommentsFromArray(resultsList);

                        if (node.hasAttribute('ns')) {
                            throw new Error('XacroParser: xacro:include name spaces not supported.');
                        }
                        const filename = evaluateAttribute(node.getAttribute('filename'), properties, true);
                        const isAbsolute = /^[/\\]/.test(filename) || /^[a-zA-Z]+:[/\\]/.test(filename);
                        const filePath = isAbsolute ? filename : currWorkingPath + filename;

                        const prevWorkingPath = currWorkingPath;
                        currWorkingPath = getUrlBase(filePath);

                        const includeContent = await loadInclude(filePath);
                        const childNodes = [...includeContent.children[0].childNodes];
                        for (const c of childNodes) {
                            await processNode(c, properties, macros, resultsList);
                        }

                        currWorkingPath = prevWorkingPath;
                        return;
                    }
                    case 'xacro:arg':
                    case 'xacro:attribute':
                    case 'xacro:element':
                        throw new Error(`XacroParser: ${ tagName } tags not supported.`);
                    default: {
                        // TODO: check if there's a 'call' attribute here which indicates that
                        // a macro should be invoked?
                        if (/^xacro:/.test(tagName) || tagName in macros) {
                            removeEndCommentsFromArray(resultsList);

                            return evaluateMacro(node, properties, macros, resultsList);
                        } else {

                            const res = node.cloneNode();
                            for (let i = 0, l = res.attributes.length; i < l; i++) {
                                const attr = res.attributes[i];
                                const value = evaluateAttribute(attr.value, properties, true);
                                res.setAttribute(attr.name, value);
                            }

                            const childNodes = [...node.childNodes];
                            const resultChildren = [];
                            for (let i = 0, l = childNodes.length; i < l; i++) {
                                await processNode(childNodes[i], properties, macros, resultChildren);
                            }
                            resultChildren.forEach(c => res.appendChild(c));
                            resultsList.push(res);
                        }
                    }

                }
            }

            // Process all property and macro tags into the objects
            async function gatherPropertiesAndMacros(el, properties, macros) {
                const propertyEl = getElementsWithName(el, 'xacro:property');
                if (!requirePrefix) {
                    propertyEl.push(...getElementsWithName(el, 'property'));
                }
                for (const el of propertyEl) {
                    await processNode(el, properties, macros);
                }

                const macroEl = getElementsWithName(el, 'xacro:macro');
                if (!requirePrefix) {
                    macroEl.push(...getElementsWithName(el, 'macro'));
                }
                for (const el of macroEl) {
                    await processNode(el, properties, macros);
                }
            }

            // Process a document node with a new property and macro scope
            async function processXacro(xacro, properties, macros) {
                const res = xacro.cloneNode();
                for (let i = 0, l = xacro.children.length; i < l; i++) {
                    const child = [];
                    await processNode(xacro.children[i], properties, macros, child);

                    const root = child[0];
                    root.removeAttribute('xmlns:xacro');
                    res.appendChild(root);
                }
                return res;
            }

            async function loadInclude(path) {

                try {
                    const text = await scope.getFileContents(path);
                    return new DOMParser().parseFromString(text, 'text/xml');
                } catch (e) {
                    throw new Error(`XacroParser: Could not load included file: ${ path }`);
                }

            }

            async function loadIncludes(xacro, workingPath, results = []) {

                const includeEl = getElementsWithName(xacro, 'xacro:include');
                if (!requirePrefix) {
                    includeEl.push(...getElementsWithName(xacro, 'include'));
                }

                const promises = includeEl.map(el => {
                    // TODO: Handle namespaces on the include.
                    if (el.hasAttribute('ns')) {
                        throw new Error('XacroParser: xacro:include name spaces not supported.');
                    }

                    const filename = el.getAttribute('filename');
                    const namespace = el.getAttribute('ns') || null;
                    const isAbsolute = /^[/\\]/.test(filename) || /^[a-zA-Z]+:[/\\]/.test(filename);
                    const filePath = isAbsolute ? filename : workingPath + filename;
                    const pr = loadInclude(filePath)
                        .then(content => {
                            results.push({ filename, namespace, content });

                            const relPath = getUrlBase(filePath);
                            return loadIncludes(content, relPath, results);
                        });
                    return pr;
                });

                await Promise.all(promises);
                return results;
            }

            // TODO: Provide a default "arg" command function that defaults to
            // xacro:arg fields.
            const scope = this;
            const inOrder = this.inOrder;

            // add a file separator to the end of the working path if it's specified
            // and doesn't have one.
            const workingPath = this.workingPath + (this.workingPath && !/[\\/]$/.test(this.workingPath) ? '/' : '');
            const requirePrefix = this.requirePrefix;
            const rospackCommands = this.rospackCommands;
            const globalMacros = {};
            const includeMap = {};
            const globalProperties = { True: 1, False: 0 };
            globalProperties[PARENT_SCOPE] = globalProperties;

            // TODO: remove unsave eval
            const handleRospackCommand = (stem, ...args) => rospackCommands[stem](...args);
            const handleExpressionEvaluation = evaluateExpression;

            let localProperties = this.localProperties;
            let currWorkingPath = workingPath;
            let content = new DOMParser().parseFromString(data, 'text/xml');

            if (localProperties && !inOrder) {
                console.warn('XacroParser: Implicitly setting "localProperties" option to false because "inOrder" is false.');
                localProperties = false;
            }

            let inOrderPromise = null;
            if (!inOrder) {
                inOrderPromise = (async function() {
                    await gatherPropertiesAndMacros(content, globalProperties, globalMacros);
                    content = deepClone(content, true);

                    return loadIncludes(content, workingPath)
                        .then(arr => {
                            arr.forEach(inc => {
                                // TODO: handle namespaces here when rolling up properties and macros
                                gatherPropertiesAndMacros(inc.content, globalProperties, globalMacros);
                                inc.content = deepClone(inc.content, true);
                                includeMap[inc.filename] = inc.content;
                            });
                        });
                })();
            } else {
                inOrderPromise = Promise.resolve();
            }

            await inOrderPromise;
            return processXacro(content, globalProperties, globalMacros);
        }

    }

    class XacroLoader extends XacroParser {

        constructor() {

            super();
            this.fetchOptions = {};

        }

        load(url, onComplete, onError) {

            const workingPath = getUrlBase(url);
            if (this.workingPath === '') {

                this.workingPath = workingPath;

            }

            this
                .getFileContents(url)
                .then(text => {

                    this.parse(text, onComplete, onError);

                })
                .catch(e => {

                    if (onError) {

                        onError(e);

                    }

                });

        }

        parse(data, onComplete, onError) {

            super
                .parse(data)
                .then(onComplete)
                .catch(e => {

                    if (onError) {

                        onError(e);

                    }

                });

        }

        getFileContents(path) {

            return fetch(path, this.fetchOptions).then(res => res.text());

        }

    }

    exports.XacroLoader = XacroLoader;
    exports.XacroParser = XacroParser;

    Object.defineProperty(exports, '__esModule', { value: true });

})));
//# sourceMappingURL=index.js.map
