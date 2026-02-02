import js from '@eslint/js';
import vitest from '@vitest/eslint-plugin';
import globals from 'globals';
import mdcs from 'eslint-config-mdcs';

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
		},
	},
];
