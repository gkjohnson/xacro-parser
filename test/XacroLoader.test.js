import { XacroLoader } from '../src/XacroLoader.js';
import { JSDOM } from 'jsdom';

function unformat( xml ) {

	return xml.replace( />\s+</g, '><' ).trim();

}

const files = {
	'./a.xacro':
		`<?xml version="1.0"?>
		<robot xmlns:xacro="http://ros.org/wiki/xacro">
			<xacro:property name="a" value="a-val"/>
			<xacro:property name="b" value="b-val"/>
			<xacro:macro name="test-macro" params="b">
				<child b="\${b}"/>
				<child b="\${b}"/>
			</xacro:macro>
			<inlined/>
		</robot>
	`,
	'./b.xacro':
		`<?xml version="1.0"?>
		<robot xmlns:xacro="http://ros.org/wiki/xacro">
			<xacro:include filename="./c.xacro"/>
			<inlined-b/>
		</robot>
	`,
	'./c.xacro':
		`<?xml version="1.0"?>
		<robot xmlns:xacro="http://ros.org/wiki/xacro">
			<inlined-c/>
		</robot>
	`,
	'./d.xacro':
		`<?xml version="1.0"?>
		<robot xmlns:xacro="http://ros.org/wiki/xacro">
			<child d="$(arg d)"/>
		</robot>
	`,

};

beforeEach( () => {

	const jsdom = new JSDOM();
	const window = jsdom.window;
	global.DOMParser = window.DOMParser;
	global.XMLSerializer = window.XMLSerializer;
	global.fetch = function ( url ) {

		url = url.replace( /^(\.\/)+/, './' );
		return Promise.resolve( {
			ok: url in files,
			text() {

				if ( url in files ) {

					return Promise.resolve( files[ url ] );

				} else {

					return Promise.reject( new Error() );

				}

			},
		} );

	};

} );

describe( 'XacroLoader', () => {

	describe( 'error callback', () => {

		it( 'should get called if the file couldn\'t be loaded', () => new Promise( ( resolve ) => {

			const loader = new XacroLoader();
			loader.load( 'nonexistent.xacro', () => {}, () => resolve() );

		} ) );

		it( 'should get called if a child file couldn\'t be loaded', () => new Promise( ( resolve ) => {

			const loader = new XacroLoader();
			loader.parse(
				`<?xml version="1.0"?>
				<robot xmlns:xacro="http://ros.org/wiki/xacro">
					<xacro:include filename="./nonexistent.xacro"/>
					<inlined-b/>
				</robot>
				`,
				() => {},
				() => resolve(),
			);

		} ) );

	} );

	describe( 'properties', () => {

		it( 'should replace property values in attributes.', () => {

			const content =
				`<?xml version="1.0"?>
				<robot xmlns:xacro="http://ros.org/wiki/xacro">
					<xacro:property name="test" value="val"/>
					<link name="\${test}_link">
						<child attr="_\${test}_\${test}"/>
					</link>
				</robot>
			`;
			const loader = new XacroLoader();
			return new Promise( ( resolve, reject ) => loader.parse(
				content, res => {

					const str = new XMLSerializer().serializeToString( res );
					expect( unformat( str ) ).toEqual( unformat(
						`<robot>
							<link name="val_link">
								<child attr="_val_val"/>
							</link>
						</robot>`,
					) );
					resolve();

				},
				reject,
			) );

		} );

		it( 'should expand property blocks.', () => {

			const content =
				`<?xml version="1.0"?>
				<robot xmlns:xacro="http://ros.org/wiki/xacro">
					<xacro:property name="test">
						<a/>
						<b/>
					</xacro:property>

					<xacro:insert_block name="test"/>
				</robot>
			`;
			const loader = new XacroLoader();
			return new Promise( ( resolve, reject ) => loader.parse(
				content, res => {

					const str = new XMLSerializer().serializeToString( res );
					expect( unformat( str ) ).toEqual( unformat(
						`<robot>
							<a/>
							<b/>
						</robot>`,
					) );
					resolve();

				},
				reject,
			) );

		} );

	} );

	describe( 'comments', () => {

		it( 'should be stripped before xacro elements', () => {

			const content =
				`<?xml version="1.0"?>
				<robot xmlns:xacro="http://ros.org/wiki/xacro">
					<!--third comment before property -->

					<!--second comment before property -->
					<!--comment before property -->
					<xacro:property name="test" value="val"/>

					<!--comment before link-->
					<link name="\${test}_link">
						<child attr="_\${test}_\${test}"/>
					</link>
				</robot>
			`;

			const loader = new XacroLoader();
			return new Promise( ( resolve, reject ) => loader.parse( content, res => {

				const str = new XMLSerializer().serializeToString( res );
				expect( unformat( str ) ).toEqual( unformat(
					`<robot>
					<!--comment before link-->
					<link name="val_link">
						<child attr="_val_val"/>
					</link>
				</robot>`,
				) );

				resolve();

			}, reject ) );

		} );

		it( 'should not process commands in comments.', () => {

			const content =
				`<?xml version="1.0"?>
				<robot xmlns:xacro="http://ros.org/wiki/xacro">
					<!--
					<link name="\${test}_link">
						<child attr="_\${test}_\${test}"/>
					</link>
					-->
				</robot>
			`;

			const loader = new XacroLoader();
			return new Promise( ( resolve, reject ) => loader.parse( content, res => {

				const str = new XMLSerializer().serializeToString( res );
				expect( unformat( str ) ).toEqual( unformat(
					`<robot>
					<!--
					<link name="\${test}_link">
						<child attr="_\${test}_\${test}"/>
					 </link>
					-->
					</robot>`,
				) );

				resolve();

			}, reject ) );

		} );

	} );

	describe( 'macros', () => {

		it( 'should expand a basic macro.', () => {

			const content =
				`<?xml version="1.0"?>
				<robot xmlns:xacro="http://ros.org/wiki/xacro">
					<xacro:macro name="test">
						<macro-child attr="test"/>
						<macro-child2 attr="test">
							<child/>
						</macro-child2>
					</xacro:macro>
					<xacro:test/>
				</robot>
			`;
			const loader = new XacroLoader();
			return new Promise( ( resolve, reject ) => loader.parse(
				content, res => {

					const str = new XMLSerializer().serializeToString( res );
					expect( unformat( str ) ).toEqual( unformat(
						`<robot>
							<macro-child attr="test"/>
							<macro-child2 attr="test">
								<child/>
							</macro-child2>
						</robot>`,
					) );
					resolve();

				},
				reject,
			) );

		} );

		it( 'should expand a macro with inputs.', () => {

			const content =
				`<?xml version="1.0"?>
				<robot xmlns:xacro="http://ros.org/wiki/xacro">
					<xacro:macro name="test" params="a b c:=10">
						<child a="\${a}" b="\${b}" c="\${c}"/>
					</xacro:macro>
					<xacro:test a="1" b="2"/>
					<xacro:test a="1" b="2" c="3"/>
				</robot>
			`;
			const loader = new XacroLoader();
			return new Promise( ( resolve, reject ) => loader.parse(
				content, res => {

					const str = new XMLSerializer().serializeToString( res );
					expect( unformat( str ) ).toEqual( unformat(
						`<robot>
							<child a="1" b="2" c="10"/>
							<child a="1" b="2" c="3"/>
						</robot>`,
					) );
					resolve();

				},
				reject,
			) );

		} );

		it( 'should resolve params and defaults with properties.', () => {

			const content =
				`<?xml version="1.0"?>
				<robot xmlns:xacro="http://ros.org/wiki/xacro">
					<xacro:property name="input" value="10"/>
					<xacro:macro name="test" params="a b c:=\${input}">
						<child a="\${a}" b="\${b}" c="\${c}"/>
					</xacro:macro>
					<xacro:test a="1" b="\${input}"/>
					<xacro:test a="1" b="\${input}" c="3"/>
				</robot>
			`;
			const loader = new XacroLoader();
			return new Promise( ( resolve, reject ) => loader.parse(
				content, res => {

					const str = new XMLSerializer().serializeToString( res );
					expect( unformat( str ) ).toEqual( unformat(
						`<robot>
							<child a="1" b="10" c="10"/>
							<child a="1" b="10" c="3"/>
						</robot>`,
					) );
					resolve();

				},
				reject,
			) );

		} );

		it( 'should resolve params and defaults with string properties.', () => {

			const content =
				`<?xml version="1.0"?>
				<robot xmlns:xacro="http://ros.org/wiki/xacro">
					<xacro:property name="input" value="10"/>
					<xacro:macro name="test" params="a:='1.0 2.0 3.0' b:='1.0 2.0'">
						<child a="\${a}" b="\${b}"/>
					</xacro:macro>
					<xacro:test/>
					<xacro:test a="1"/>
				</robot>
			`;
			const loader = new XacroLoader();
			return new Promise( ( resolve, reject ) => loader.parse(
				content, res => {

					const str = new XMLSerializer().serializeToString( res );
					expect( unformat( str ) ).toEqual( unformat(
						`<robot>
							<child a="1.0 2.0 3.0" b="1.0 2.0"/>
							<child a="1" b="1.0 2.0"/>
						</robot>`,
					) );
					resolve();

				},
				reject,
			) );

		} );

		it( 'should throw an error if a parameter cannot be resolved.', () => {

			const content =
				`<?xml version="1.0"?>
				<robot xmlns:xacro="http://ros.org/wiki/xacro">
					<xacro:property name="input" value="10"/>
					<xacro:macro name="test" params="a b c:=\${input}">
						<child a="\${a}" b="\${b}" c="\${c}"/>
					</xacro:macro>
					<other c="\${c}"/>
				</robot>
			`;
			const loader = new XacroLoader();
			return new Promise( ( resolve, reject ) => loader.parse(
				content, () => reject( new Error( 'Should have thrown' ) ), () => resolve(),
			) );

		} );

		it( 'should expand include blocks.', () => {

			const content =
				`<?xml version="1.0"?>
				<robot xmlns:xacro="http://ros.org/wiki/xacro">
					<xacro:macro name="test" params="*a **b **c">
						<xacro:insert_block name="a"/>
						<xacro:insert_block name="b"/>
						<xacro:insert_block name="c"/>
					</xacro:macro>
					<xacro:test>
						<first/>
						<b>
							<second/>
						</b>
						<ignored/>
						<c>
							<third/>
							<fourth/>
						</c>
					</xacro:test>
				</robot>
			`;
			const loader = new XacroLoader();
			return new Promise( ( resolve, reject ) => loader.parse(
				content, res => {

					const str = new XMLSerializer().serializeToString( res );
					expect( unformat( str ) ).toEqual( unformat(
						`<robot>
							<first/>
							<second/>
							<third/>
							<fourth/>
						</robot>`,
					) );
					resolve();

				},
				reject,
			) );

		} );

		it( 'should expand the body of the target before running the macro.', () => {

			const content =
				`<?xml version="1.0"?>
				<robot xmlns:xacro="http://ros.org/wiki/xacro">
					<xacro:macro name="test" params="*a *b">
						<xacro:insert_block name="a"/>
						<xacro:insert_block name="b"/>
					</xacro:macro>
					<xacro:test>
						<xacro:if value="true">
							<a/>
						</xacro:if>
						<xacro:unless value="true">
							<b/>
						</xacro:unless>
						<c/>
					</xacro:test>
				</robot>
			`;
			const loader = new XacroLoader();
			return new Promise( ( resolve, reject ) => loader.parse(
				content, res => {

					const str = new XMLSerializer().serializeToString( res );
					expect( unformat( str ) ).toEqual( unformat(
						`<robot>
							<a/>
							<c/>
						</robot>`,
					) );
					resolve();

				},
				reject,
			) );

		} );

		it( 'should handle element references after the first parameters.', () => {

			const content =
				`<?xml version="1.0"?>
				<robot xmlns:xacro="http://ros.org/wiki/xacro">
					<xacro:macro name="test" params="x y z *a *b">
						<xacro:insert_block name="a"/>
						<xacro:insert_block name="b"/>
						<child x="\${x}" y="\${y}" z="\${z}"/>
					</xacro:macro>
					<xacro:test x="1" y="2" z="3">
						<a/>
						<b/>
					</xacro:test>
				</robot>
			`;
			const loader = new XacroLoader();
			return new Promise( ( resolve, reject ) => loader.parse(
				content, res => {

					const str = new XMLSerializer().serializeToString( res );
					expect( unformat( str ) ).toEqual( unformat(
						`<robot>
							<a/>
							<b/>
							<child x="1" y="2" z="3"/>
						</robot>`,
					) );
					resolve();

				},
				reject,
			) );

		} );

		it( 'should expand macros recursively.', () => {

			const content =
				`<?xml version="1.0"?>
				<robot xmlns:xacro="http://ros.org/wiki/xacro">
					<xacro:macro name="outter" params="a">
						<xacro:macro name="inner" params="b">
							<result c="\${b}"/>
						</xacro:macro>
						<xacro:inner b="\${a}"/>
						<after/>
					</xacro:macro>
					<xacro:outter a="100"/>
				</robot>
			`;
			const loader = new XacroLoader();
			return new Promise( ( resolve, reject ) => loader.parse(
				content, res => {

					const str = new XMLSerializer().serializeToString( res );
					expect( unformat( str ) ).toEqual( unformat(
						`<robot>
							<result c="100"/>
							<after/>
						</robot>`,
					) );
					resolve();

				},
				reject,
			) );

		} );

		it( 'should pass insert_block commands down through nested macros.', () => {

			const content =
				`<?xml version="1.0"?>
				<robot xmlns:xacro="http://ros.org/wiki/xacro">
					<xacro:macro name="inner" params="**b">
						<xacro:insert_block name="b"/>
					</xacro:macro>
					<xacro:macro name="outter" params="**a">
						<xacro:inner>
							<b>
								<xacro:insert_block name="a"/>
							</b>
						</xacro:inner>
						<after/>
					</xacro:macro>
					<xacro:outter>
						<a>
							<child/>
						</a>
					</xacro:outter>
				</robot>
			`;
			const loader = new XacroLoader();
			return new Promise( ( resolve, reject ) => loader.parse(
				content, res => {

					const str = new XMLSerializer().serializeToString( res );
					expect( unformat( str ) ).toEqual( unformat(
						`<robot>
							<child/>
							<after/>
						</robot>`,
					) );
					resolve();

				},
				reject,
			) );

		} );

		it.todo( 'should support pass through arguments with defaults using the :=^|val notation.' );

	} );

	describe( 'condition blocks', () => {

		it( 'should expand conditional statements correctly.', () => {

			const content =
				`<?xml version="1.0"?>
				<robot xmlns:xacro="http://ros.org/wiki/xacro">
					<xacro:property name="True" value="1"/>
					<xacro:property name="False" value="0"/>

					<xacro:if value="\${True}"><included/></xacro:if>
					<xacro:if value="\${False}"><excluded/></xacro:if>
					<xacro:unless value="\${True}"><excluded/></xacro:unless>
					<xacro:unless value="\${False}"><included/></xacro:unless>
				</robot>
			`;
			const loader = new XacroLoader();
			return new Promise( ( resolve, reject ) => loader.parse(
				content, res => {

					const str = new XMLSerializer().serializeToString( res );
					expect( unformat( str ) ).toEqual( unformat(
						`<robot>
							<included/>
							<included/>
						</robot>`,
					) );
					resolve();

				},
				reject,
			) );

		} );

	} );

	describe( 'expressions', () => {

		it( 'should evaluate basic expressions.', () => {

			const content =
				`<?xml version="1.0"?>
				<robot xmlns:xacro="http://ros.org/wiki/xacro">
					<xacro:property name="val" value="100"/>
					<result
						a="\${1 + 1}"
						b="\${100 / 10}"
						c="\${val / 10}"
						d="\${val*(val+.02)}"
						e="\${radians(180)/radians(90)}"
						f="\${degrees(radians(45))}"
					/>
					<result
						g="\${not 1}"
						h="\${!1}"
						i="\${1e-2}"
						j="\${true||false}"
					/>
				</robot>
			`;
			const loader = new XacroLoader();
			return new Promise( ( resolve, reject ) => loader.parse(
				content, res => {

					const str = new XMLSerializer().serializeToString( res );
					expect( unformat( str ) ).toEqual( unformat(
						`<robot>
							<result a="2" b="10" c="10" d="10002" e="2" f="45"/>
							<result g="false" h="false" i="1e-2" j="true"/>
						</robot>`,
					) );
					resolve();

				},
				reject,
			) );

		} );

		it( 'should correctly interpret negative numbers as numbers', () => {

			const content =
				`<?xml version="1.0"?>
				<robot xmlns:xacro="http://ros.org/wiki/xacro">
					<xacro:property name="val" value="-1"/>
					<xacro:property name="reflect" value="\${val}"/>
					<result
						a="\${(reflect + 1)}"
						b="\${(reflect + 1) / 2}"
						c="\${reflect}"
					/>
				</robot>
			`;
			const loader = new XacroLoader();
			return new Promise( ( resolve, reject ) => loader.parse(
				content, res => {

					const str = new XMLSerializer().serializeToString( res );
					expect( unformat( str ) ).toEqual( unformat(
						`<robot>
							<result a="0" b="0" c="-1"/>
						</robot>`,
					) );
					resolve();

				},
				reject,
			) );

		} );

		it( 'should escape dollar signs correctly', () => {

			const content =
				`<?xml version="1.0"?>
				<robot xmlns:xacro="http://ros.org/wiki/xacro">
					<xacro:property name="val" value="100"/>
					<result
						a="$\${1 + 1}"
						b="$$\${1 + 1}"
						c="$$$\${1 + 1}"
					/>
				</robot>
			`;
			const loader = new XacroLoader();
			return new Promise( ( resolve, reject ) => loader.parse(
				content, res => {

					const str = new XMLSerializer().serializeToString( res );
					expect( unformat( str ) ).toEqual( unformat(
						`<robot>
							<result a="\${1 + 1}" b="$\${1 + 1}" c="$$\${1 + 1}"/>
						</robot>`,
					) );
					resolve();

				},
				reject,
			) );

		} );

		it( 'should not allow evaluation of unsafe expressions.', async () => {

			let called = 0;
			global.testFunc = () => called ++;
			global.e = () => called ++;
			global.E = () => called ++;
			global.e123 = () => called ++;

			// these should throw errors because they can't be evaluated.
			const promises =
				[
					'testFunc()',
					'global.testFunc()',
					'$' + '{testFunc}()',
					'global.e()',
					'global.E()',
					'global[\'e\']()',
					'e()',
					'E()',
					'e123()',
				].map( exp => new Promise( ( resolve, reject ) => {

					const content =
						`<?xml version="1.0"?>
							<robot xmlns:xacro="http://ros.org/wiki/xacro">
								<xacro:property name="tf" value="testFunc" />
								<result a="\${${ exp }}"/>
							</robot>
						`;
					const loader = new XacroLoader();
					loader.parse( content, () => reject( new Error() ), () => resolve() );

				} ) );

			try {

				await Promise.all( promises );
				expect( called ).toEqual( 0 );

			} finally {

				delete global.e;
				delete global.e123;
				delete global.E;
				delete global.testFunc;

			}

		} );

	} );

	describe( 'rospack commands', () => {

		it( 'should call rospack commnds.', () => {

			const content =
				`<?xml version="1.0"?>
				<robot xmlns:xacro="http://ros.org/wiki/xacro">
					<result a="$(find-package test args)" b="$(cwd)" c="$(cwd)"/>
				</robot>
			`;

			let called = 0;

			const loader = new XacroLoader();

			loader.rospackCommands = {
				'find-package': ( ...args ) => {

					expect( args ).toEqual( [ 'test', 'args' ] );
					return 'package';

				},
				'cwd': () => {

					called ++;
					return 'cwd';

				},
			};

			return new Promise( ( resolve, reject ) => loader.parse(
				content, res => {

					const str = new XMLSerializer().serializeToString( res );
					expect( unformat( str ) ).toEqual( unformat(
						`<robot>
							<result a="package" b="cwd" c="cwd"/>
						</robot>`,
					) );
					expect( called ).toEqual( 2 );
					resolve();

				},
				reject,
			) );

		} );

		it( 'should call rospack with a function as an argument.', () => {

			const content =
				`<?xml version="1.0"?>
				<robot xmlns:xacro="http://ros.org/wiki/xacro">
					<result a="$(find-package test args)" />
				</robot>
			`;

			const loader = new XacroLoader();
			loader.rospackCommands = ( command, ...args ) => {

				expect( command ).toEqual( 'find-package' );
				expect( args ).toEqual( [ 'test', 'args' ] );
				return 'package';

			};

			return new Promise( ( resolve, reject ) => loader.parse(
				content, res => {

					const str = new XMLSerializer().serializeToString( res );
					expect( unformat( str ) ).toEqual( unformat(
						`<robot>
							<result a="package"/>
						</robot>`,
					) );
					resolve();

				},
				reject,
			) );

		} );

		it.todo( 'shoulds support rospack substitution args with default values.' );

	} );

	describe( 'include', () => {

		it( 'should import properties and macros from xacro:include tags.', () => {

			const content =
				`<?xml version="1.0"?>
				<robot xmlns:xacro="http://ros.org/wiki/xacro">
					<xacro:include filename="./a.xacro"/>
					<test a="\${a}"/>
					<xacro:test-macro b="\${b}"/>
				</robot>
			`;

			const loader = new XacroLoader();
			return new Promise( ( resolve, reject ) => loader.parse(
				content, res => {

					const str = new XMLSerializer().serializeToString( res );
					expect( unformat( str ) ).toEqual( unformat(
						`<robot>
							<inlined/>
							<test a="a-val"/>
							<child b="b-val"/>
							<child b="b-val"/>
						</robot>`,
					) );
					resolve();

				},
				reject,
			) );

		} );

		it( 'should example include blocks recursively.', () => {

			const content =
				`<?xml version="1.0"?>
				<robot xmlns:xacro="http://ros.org/wiki/xacro">
					<xacro:include filename="./b.xacro"/>
				</robot>
			`;

			const loader = new XacroLoader();
			return new Promise( ( resolve, reject ) => loader.parse(
				content, res => {

					const str = new XMLSerializer().serializeToString( res );
					expect( unformat( str ) ).toEqual( unformat(
						`<robot>
							<inlined-c/>
							<inlined-b/>
						</robot>`,
					) );
					resolve();

				},
				reject,
			) );

		} );

		it( 'should load relative paths.', () => {

			const files = {
				'http://website.com/path/./folder/a.xacro':
					`<?xml version="1.0"?>
					<robot xmlns:xacro="http://ros.org/wiki/xacro">
						<xacro:include filename="./b.xacro"/>
					</robot>
				`,
				'http://website.com/path/./folder/./b.xacro':
					`<?xml version="1.0"?>
					<robot xmlns:xacro="http://ros.org/wiki/xacro">
					</robot>
				`,
			};

			const res = [];
			global.fetch = function ( url ) {

				res.push( url );
				return Promise.resolve( {
					ok: url in files,
					text() {

						return files[ url ];

					},
				} );

			};

			const content =
				`<?xml version="1.0"?>
				<robot xmlns:xacro="http://ros.org/wiki/xacro">
					<xacro:include filename="./folder/a.xacro"/>
				</robot>
			`;

			const loader = new XacroLoader();
			loader.workingPath = 'http://website.com/path/';
			return new Promise( ( resolve, reject ) => loader.parse(
				content, () => {

					expect( res ).toEqual( [
						'http://website.com/path/./folder/a.xacro',
						'http://website.com/path/./folder/./b.xacro',
					] );
					resolve();

				},
				reject,
			) );

		} );

		it( 'should have filename attributes evaluated before including them.', () => {

			const content =
				`<?xml version="1.0"?>
				<robot xmlns:xacro="http://ros.org/wiki/xacro">
					<xacro:property name="file" value="b.xacro"/>
					<xacro:include filename="./\${file}"/>
				</robot>
			`;

			const loader = new XacroLoader();
			return new Promise( ( resolve, reject ) => loader.parse(
				content, res => {

					const str = new XMLSerializer().serializeToString( res );
					expect( unformat( str ) ).toEqual( unformat(
						`<robot>
							<inlined-c/>
							<inlined-b/>
						</robot>`,
					) );
					resolve();

				},
				reject,
			) );

		} );
		it.todo( 'should support absolute paths.' );
		it.todo( 'should respect namespaces for macros and properties.' );

	} );

	describe( 'substitution args', () => {

		it( 'should support arg substitution of different types', () => {

			const content =
				`<?xml version="1.0"?>
				<robot xmlns:xacro="http://ros.org/wiki/xacro">
					<xacro:unless value="$(arg b)">
						<child a="\${$(arg a) * 2}" c="$(arg c)" />
					</xacro:unless>
					<xacro:if value="$(arg b)">
						<child b="oops" />
					</xacro:if>
				</robot>
			`;

			const loader = new XacroLoader();
			loader.arguments = { a: 0.5, b: false, c: 'c-val' };
			return new Promise( ( resolve, reject ) => loader.parse(
				content, res => {

					const str = new XMLSerializer().serializeToString( res );
					expect( unformat( str ) ).toEqual( unformat(
						`<robot>
							<child a="1" c="c-val"/>
						</robot>`,
					) );
					resolve();

				},
				reject,
			) );

		} );
		it( 'should delegate to provided rospackCommands function, falling back to arguments', () => {

			const content =
				`<?xml version="1.0"?>
				<robot xmlns:xacro="http://ros.org/wiki/xacro">
					<child a="$(arg a)" b="$(arg b)" />
				</robot>
			`;

			const loader = new XacroLoader();
			loader.arguments = { a: 'a-val', b: 'b-val' };
			loader.rospackCommands = ( command, arg ) => {

				if ( command === 'arg' && arg === 'a' ) {

					return 'rospack-a';

				}

				return null;

			};

			return new Promise( ( resolve, reject ) => loader.parse(
				content, res => {

					const str = new XMLSerializer().serializeToString( res );
					expect( unformat( str ) ).toEqual( unformat(
						`<robot>
							<child a="rospack-a" b="b-val"/>
						</robot>`,
					) );
					resolve();

				},
				reject,
			) );

		} );
		it( 'should delegate to provided rospackCommands object, falling back to arguments', () => {

			const content =
				`<?xml version="1.0"?>
				<robot xmlns:xacro="http://ros.org/wiki/xacro">
					<child a="$(arg a)" b="$(arg b)" />
				</robot>
			`;

			const loader = new XacroLoader();
			loader.arguments = { a: 'a-val', b: 'b-val' };
			loader.rospackCommands = {
				arg: ( arg ) => {

					if ( arg === 'a' ) {

						return 'rospack-a';

					}

					return null;

				},
			};

			return new Promise( ( resolve, reject ) => loader.parse(
				content, res => {

					const str = new XMLSerializer().serializeToString( res );
					expect( unformat( str ) ).toEqual( unformat(
						`<robot>
							<child a="rospack-a" b="b-val"/>
						</robot>`,
					) );
					resolve();

				},
				reject,
			) );

		} );
		it( 'should make provided args accessible to imported files.', () => {

			const content =
				`<?xml version="1.0"?>
				<robot xmlns:xacro="http://ros.org/wiki/xacro">
					<xacro:include filename="./d.xacro"/>
				</robot>
			`;

			return new Promise( ( resolve, reject ) => {

				const loader = new XacroLoader();
				loader.arguments = { d: 'd-val' };
				loader.parse(
					content, res => {

						const str = new XMLSerializer().serializeToString( res );
						expect( unformat( str ) ).toEqual( unformat(
							`<robot>
								<child d="d-val"/>
							</robot>`,
						) );
						resolve();

					},
					reject,
				);

			} );

		} );
		it( 'should make xacro:arg defaults accessible to imported files.', () => {

			const content =
				`<?xml version="1.0"?>
				<robot xmlns:xacro="http://ros.org/wiki/xacro">
					<xacro:arg name="d" default="d-val"/>
					<xacro:include filename="./d.xacro"/>
				</robot>
			`;

			return new Promise( ( resolve, reject ) => {

				const loader = new XacroLoader();
				loader.parse(
					content, res => {

						const str = new XMLSerializer().serializeToString( res );
						expect( unformat( str ) ).toEqual( unformat(
							`<robot>
								<child d="d-val"/>
							</robot>`,
						) );
						resolve();

					},
					reject,
				);

			} );

		} );

	} );

	describe( 'options.inOrder', () => {

		it( 'should process properties and macros in order if true', () => {

			const content =
				`<?xml version="1.0"?>
				<robot xmlns:xacro="http://ros.org/wiki/xacro">
					<xacro:property name="x" value="100"/>
					<xacro:if value="1">
						<xacro:property name="y" value="200"/>
					</xacro:if>
					<xacro:if value="1">
						<xacro:property name="z" value="500"/>
					</xacro:if>
					<xacro:if value="0">
						<xacro:property name="z" value="300"/>
					</xacro:if>
					<result x="\${x}" y="\${y}" z="\${z}"/>

					<xacro:macro name="test"><a/></xacro:macro>
					<xacro:test/>
					<xacro:macro name="test"><b/></xacro:macro>
					<xacro:test/>
				</robot>
			`;

			return new Promise( ( resolve, reject ) => {

				const loader = new XacroLoader();
				loader.localProperties = false;
				loader.inOrder = true;
				loader.parse(
					content, res => {

						const str = new XMLSerializer().serializeToString( res );
						expect( unformat( str ) ).toEqual( unformat(
							`<robot>
								<result x="100" y="200" z="500"/>
								<a/>
								<b/>
							</robot>`,
						) );
						resolve();

					},
					reject,
				);

			} );

		} );

		it( 'should throw an error if true but a property is out of order', () => {

			const content =
				`<?xml version="1.0"?>
				<robot xmlns:xacro="http://ros.org/wiki/xacro">
					<result x="\${x}" y="\${y}" z="\${z}"/>
					<xacro:property name="x" value="100"/>
					<xacro:if value="1">
						<xacro:property name="y" value="200"/>
					</xacro:if>
					<xacro:if value="0">
						<xacro:property name="z" value="300"/>
					</xacro:if>
				</robot>
			`;

			return new Promise( ( resolve, reject ) => {

				const loader = new XacroLoader();
				loader.localProperties = false;
				loader.inOrder = true;
				loader.parse(
					content, () => reject( new Error( 'Should have thrown' ) ), () => resolve(),
				);

			} );

		} );

		it( 'should process properties and macros first if false', () => {

			const content =
				`<?xml version="1.0"?>
				<robot xmlns:xacro="http://ros.org/wiki/xacro">
					<result x="\${x}" y="\${y}" z="\${z}"/>
					<xacro:property name="x" value="100"/>
					<xacro:if value="1">
						<xacro:property name="y" value="200"/>
					</xacro:if>
					<xacro:if value="0">
						<xacro:property name="z" value="300"/>
					</xacro:if>
					<result x="\${x}" y="\${y}" z="\${z}"/>

					<xacro:macro name="test"><a/></xacro:macro>
					<xacro:test/>
					<xacro:macro name="test"><b/></xacro:macro>
					<xacro:test/>
				</robot>
			`;

			return new Promise( ( resolve, reject ) => {

				const loader = new XacroLoader();
				loader.localProperties = false;
				loader.inOrder = false;
				loader.parse(
					content, res => {

						const str = new XMLSerializer().serializeToString( res );
						expect( unformat( str ) ).toEqual( unformat(
							`<robot>
								<result x="100" y="200" z="300"/>
								<result x="100" y="200" z="300"/>
								<b/>
								<b/>
							</robot>`,
						) );
						resolve();

					},
					reject,
				);

			} );

		} );

		it( 'should process the properties and macros in included files in order.', () => {

			const content =
				`<?xml version="1.0"?>
				<robot xmlns:xacro="http://ros.org/wiki/xacro">
					<xacro:property name="a" value="aa"/>
					<xacro:macro name="test-macro">
						<child/>
					</xacro:macro>

					<test a="\${a}"/>
					<xacro:test-macro/>

					<xacro:include filename="./a.xacro"/>
					<test a="\${a}"/>
					<xacro:test-macro b="\${b}"/>
				</robot>
			`;

			return new Promise( ( resolve, reject ) => {

				const loader = new XacroLoader();
				loader.inOrder = true;
				loader.parse(
					content, res => {

						const str = new XMLSerializer().serializeToString( res );
						expect( unformat( str ) ).toEqual( unformat(
							`<robot>
								<test a="aa"/>
								<child/>

								<inlined/>
								<test a="a-val"/>
								<child b="b-val"/>
								<child b="b-val"/>
							</robot>`,
						) );
						resolve();

					},
					reject,
				);

			} );

		} );

	} );

	describe( 'options.localProperties', () => {

		it( 'should scope properties to macros if true.', () => {

			const content =
				`<?xml version="1.0"?>
				<robot xmlns:xacro="http://ros.org/wiki/xacro">
					<xacro:macro name="test">
						<xacro:property name="input" value="100"/>
						<xacro:property name="globalInput" value="200" scope="global"/>
						<result value="\${input}"/>
					</xacro:macro>
					<xacro:test/>
					<after value="\${input}"/>
					<after value="\${globalInput}"/>
				</robot>
			`;

			return new Promise( ( resolve, reject ) => {

				const loader = new XacroLoader();
				loader.localProperties = true;
				loader.inOrder = true;
				loader.parse(
					content, () => reject( new Error( 'Should have thrown' ) ), () => resolve(),
				);

			} );

		} );

		it( 'should not scope properties to macros if false.', () => {

			const content =
				`<?xml version="1.0"?>
				<robot xmlns:xacro="http://ros.org/wiki/xacro">
					<xacro:macro name="test">
						<xacro:property name="input" value="100"/>
						<result value="\${input}"/>
					</xacro:macro>
					<xacro:test/>
					<after value="\${input}"/>
				</robot>
			`;

			return new Promise( ( resolve, reject ) => {

				const loader = new XacroLoader();
				loader.localProperties = false;
				loader.inOrder = true;
				loader.parse(
					content, res => {

						const str = new XMLSerializer().serializeToString( res );
						expect( unformat( str ) ).toEqual( unformat(
							`<robot>
								<result value="100"/>
								<after value="100"/>
							</robot>`,
						) );
						resolve();

					},
					reject,
				);

			} );

		} );

	} );

	describe( 'options.requirePrefix', () => {

		it( 'should require the prefix if true', () => {

			const content = `
					<robot xmlns:xacro="http://ros.org/wiki/xacro">
						<include filename="./a.xacro"/>
						<property name="input" value="100"/>
						<property name="block"><a/></property>

						<if value="1">100</if>
						<unless value="1">200</unless>

						<macro name="test">
							<result value="\${input}"/>
						</macro>

						<test/>
						<insert_block name="block"/>
						<after value="\${input}"/>
					</robot>
				`;

			// this should throw an error because a property is not defined.
			return new Promise( ( resolve, reject ) => {

				const loader = new XacroLoader();
				loader.requirePrefix = true;
				loader.parse(
					content, () => reject( new Error( 'Should have thrown' ) ), () => resolve(),
				);

			} );

		} );

		it( 'should not require the prefix if false', () => {

			const content = `
					<robot xmlns:xacro="http://ros.org/wiki/xacro">
						<include filename="./a.xacro"/>
						<property name="input" value="100"/>
						<property name="block"><a/></property>

						<if value="1"><a>100</a></if>
						<unless value="1"><b>200</b></unless>

						<macro name="test">
							<result value="\${input}"/>
						</macro>

						<test/>
						<insert_block name="block"/>
						<after/>
					</robot>
				`;

			return new Promise( ( resolve, reject ) => {

				const loader = new XacroLoader();
				loader.requirePrefix = false;
				loader.parse(
					content, res => {

						const str = new XMLSerializer().serializeToString( res );
						expect( unformat( str ) ).toEqual( unformat( `
							<robot>
								<inlined/>
								<a>100</a>
								<result value="100"/>
								<a/>
								<after/>
							</robot>
						` ) );
						resolve();

					},
					reject,
				);

			} );

		} );

	} );

	it.todo( 'test absolute path resolution for cases with "/", "C:/", "C:\\", "https://", etc' );

} );
