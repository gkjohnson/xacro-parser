import js from '@eslint/js';
import vitest from '@vitest/eslint-plugin';
import globals from 'globals';
import mdcs from 'eslint-config-mdcs';
import jsdoc from 'eslint-plugin-jsdoc';

export default [
	// files to ignore
	{
		name: 'files to ignore',
		ignores: [
			'**/node_modules/**',
			'**/umd/**',
		],
	},

	// recommended
	js.configs.recommended,

	// base rules
	{
		name: 'base rules',
		files: [ '**/*.js' ],
		languageOptions: {
			ecmaVersion: 2020,
			sourceType: 'module',
			globals: {
				...globals.browser,
				...globals.node,
			},
		},
		rules: {
			...mdcs.rules,
			'no-mixed-spaces-and-tabs': 'error',
		},
	},

	// jsdoc
	{
		name: 'jsdoc rules',
		files: [ '**/*.js' ],
		plugins: {
			jsdoc,
		},
		settings: {
			jsdoc: {
				preferredTypes: {
					Any: 'any',
					Boolean: 'boolean',
					Number: 'number',
					object: 'Object',
					String: 'string',
				},
				tagNamePreference: {
					return: 'returns',
					augments: 'extends',
					classdesc: false,
				},
			},
		},
		rules: {
			'jsdoc/check-tag-names': [ 'error', { definedTags: [ 'warn', 'note', 'section' ] } ],
			'jsdoc/check-types': 'error',
			'jsdoc/no-undefined-types': [ 'error', {
				definedTypes: [ 'RequestInit' ],
			} ],
			'jsdoc/require-param-type': 'error',
			'jsdoc/require-returns-type': 'error',
			'jsdoc/require-returns': 'off',
			'jsdoc/require-param-description': 'off',
			'jsdoc/require-returns-description': 'off',
		},
	},

	// vitest
	{
		name: 'vitest rules',
		files: [ '**/*.test.js' ],
		plugins: {
			vitest,
		},
		languageOptions: {
			globals: {
				...vitest.environments.env.globals,
			},
		},
		rules: {
			...vitest.configs.recommended.rules,
			'vitest/valid-describe-callback': 0,
			'vitest/expect-expect': 0,
		},
	},
];
