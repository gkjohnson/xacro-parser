// Tests from https://github.com/ros/xacro/blob/melodic-devel/test/test_xacro.py
import { XacroLoader } from '../src/XacroLoader.js';
import { JSDOM } from 'jsdom';
import { unformat } from './utils.js';

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

};

beforeEach(() => {
    const jsdom = new JSDOM();
    const window = jsdom.window;
    global.DOMParser = window.DOMParser;
    global.XMLSerializer = window.XMLSerializer;
    global.fetch = function(url) {
        url = url.replace(/^(\.\/)+/, './');
        return Promise.resolve({
            text() {
                return Promise.resolve(files[url]);
            },
        });
    };
});

function runXacro(content, done, options, onError) {
    onError = onError || (err => {
        throw err;
    });
    return new Promise(resolve => {
        const loader = new XacroLoader();
        Object.assign(loader, options);
        loader.parse(
            content, res => {
                const str = new XMLSerializer().serializeToString(res);
                done(unformat(str));
                resolve();
            },
            err => {
                onError(err);
            },
        );
    });

}

describe('Basic Xacro Tests', () => {
    it('test_xacro_element', async() =>
        runXacro(
            `<xml xmlns:xacro="http://www.ros.org/wiki/xacro">
                <xacro:property name="foo" value="1.0"/>
                <xacro:macro name="m" params="foo"><a foo="\${foo}"/></xacro:macro>
                <xacro:m foo="1 \${foo}"/>
                <!-- now redefining the property and macro -->
                <xacro:property name="foo" value="2.0"/>
                <xacro:macro name="m" params="foo"><b bar="\${foo}"/></xacro:macro>
                <xacro:m foo="2 \${foo}"/>
            </xml>`,
            res => {
                expect(res).toEqual(unformat(`
                <xml>
                    <a foo="1 1.0"/>
                    <b bar="2 2.0"/>
                </xml>
                `));
            },
        ),
    );

    it('test_should_replace_before_macroexpand', async() =>
        runXacro(
            `<a xmlns:xacro="http://www.ros.org/wiki/xacro">
                <xacro:macro name="inner" params="*the_block">
                    <in_the_inner>
                        <xacro:insert_block name="the_block" />
                    </in_the_inner>
                </xacro:macro>
                <xacro:macro name="outer" params="*the_block">
                    <in_the_outer>
                        <xacro:inner><xacro:insert_block name="the_block" /></xacro:inner>
                    </in_the_outer>
                </xacro:macro>
                <xacro:outer>
                    <woot />
                </xacro:outer>
            </a>`,
            res => {
                expect(res).toEqual(unformat(`
                <a>
                    <in_the_outer>
                        <in_the_inner>
                            <woot/>
                        </in_the_inner>
                    </in_the_outer>
                </a>
                `));
            },
        ),
    );

    it.todo('test_evaluate_macro_params_before_body');

    it('test_macro_params_escaped_string', async() =>
        runXacro(
            `<a xmlns:xacro="http://www.ros.org/wiki/xacro">
                <xacro:macro name="foo" params="a='1 -2' c=3">
                    <bar a="\${a}" c="\${c}"/>
                </xacro:macro>
                <xacro:foo/>
            </a>`,
            res => {
                expect(res).toEqual(unformat(`
                <a>
                    <bar a="1 -2" c="3"/>
                </a>
                `));
            },
        ),
    );

    it('test_property_replacement', async() =>
        runXacro(
            `<a xmlns:xacro="http://www.ros.org/wiki/xacro">
                <xacro:property name="foo" value="42" />
                <the_foo result="\${foo}" />
            </a>`,
            res => {
                expect(res).toEqual(unformat(`
                <a>
                    <the_foo result="42"/>
                </a>
                `));
            },
        ),
    );

    it('test_property_scope_parent', async() =>
        runXacro(
            `<a xmlns:xacro="http://www.ros.org/wiki/xacro">
                <xacro:macro name="foo" params="factor">
                    <xacro:property name="foo" value="\${21*factor}" scope="parent"/>
                </xacro:macro>
                <xacro:foo factor="2"/>
                <a foo="\${foo}"/>
            </a>`,
            res => {
                expect(res).toEqual(unformat(`
                <a>
                    <a foo="42"/>
                </a>
                `));
            },
        ),
    );

    it('test_property_scope_global', async() =>
        runXacro(
            `<a xmlns:xacro="http://www.ros.org/wiki/xacro">
                <xacro:macro name="foo" params="factor">
                    <xacro:macro name="bar">
                        <xacro:property name="foo" value="\${21*factor}" scope="global"/>
                    </xacro:macro>
                    <xacro:bar/>
                </xacro:macro>
                <xacro:foo factor="2"/>
                <a foo="\${foo}"/>
            </a>`,
            res => {
                expect(res).toEqual(unformat(`
                <a>
                    <a foo="42"/>
                </a>
                `));
            },
        ),
    );

    it('test_math_ignores_spaces', async() =>
        runXacro(
            `<a><f v="\${0.9 / 2 - 0.2}" /></a>`,
            res => {
                expect(res).toEqual(unformat(`
                <a>
                    <f v="0.25"/>
                </a>
                `));
            },
        ),
    );

    it.todo('test_substitution_args_find');
    it('test_substitution_args_arg', async() =>
        runXacro(
            `<a><f v="$(arg sub_arg)" /></a>`,
            res => {
                expect(res).toEqual(unformat(`
            <a><f v="my_arg"/></a>
            `));
            },
            {arguments: {sub_arg: 'my_arg'}},
        ),
    );

    it('test_escaping_dollar_braces', async() =>
        runXacro(
            `<a b="$\${foo}" c="$$\${foo}" d="text $\${foo}" e="text $$\${foo}" f="$$(pwd)" />`,
            res => {
                expect(res).toEqual(unformat(`
                <a b="\${foo}" c="$\${foo}" d="text \${foo}" e="text $\${foo}" f="$(pwd)"/>
                `));
            },
        ),
    );

    it('test_just_a_dollar_sign', async() =>
        runXacro(
            `<a b="$" c="text $" d="text $ text"/>`,
            res => {
                expect(res).toEqual(unformat(`
                <a b="$" c="text $" d="text $ text"/>
                `));
            },
        ),
    );

    it('test_multiple_insert_blocks', async() =>
        runXacro(
            `<a xmlns:xacro="http://www.ros.org/wiki/xacro">
                <xacro:macro name="foo" params="*block">
                    <xacro:insert_block name="block" />
                    <xacro:insert_block name="block" />
                </xacro:macro>
                <xacro:foo>
                    <a_block />
                </xacro:foo>
            </a>`,
            res => {
                expect(res).toEqual(unformat(`
                    <a>
                        <a_block/>
                        <a_block/>
                    </a>
                `));
            },
        ),
    );

    it('test_multiple_insert_blocks', async() => {
        await runXacro(
            `<a xmlns:xacro="http://www.ros.org/wiki/xacro">
                <xacro:macro name="foo" params="*block1 *block2">
                    <xacro:insert_block name="block1" />
                    <xacro:insert_block name="block2" />
                </xacro:macro>
                <xacro:foo>
                    <block1/>
                    <block2/>
                </xacro:foo>
            </a>`,
            res => {
                expect(res).toEqual(unformat(`
                    <a>
                        <block1/>
                        <block2/>
                    </a>
                `));
            },
        );

        await runXacro(
            `<a xmlns:xacro="http://www.ros.org/wiki/xacro">
                <xacro:macro name="foo" params="*block2 *block1">
                    <xacro:insert_block name="block1" />
                    <xacro:insert_block name="block2" />
                </xacro:macro>
                <xacro:foo>
                    <block1/>
                    <block2/>
                </xacro:foo>
            </a>`,
            res => {
                expect(res).toEqual(unformat(`
                    <a>
                        <block2/>
                        <block1/>
                    </a>
                `));
            },
        );

    });

    it('test_integer_stays_integer', async() =>
        runXacro(
            `<a xmlns:xacro="http://www.ros.org/wiki/xacro">
                <xacro:macro name="m" params="num">
                    <test number="\${num}" />
                </xacro:macro>
                <xacro:m num="100" />
            </a>`,
            res => {
                expect(res).toEqual(unformat(`
                    <a>
                        <test number="100"/>
                    </a>
                `));
            },
        ),
    );

    it('test_insert_block_property', async() =>
        runXacro(
            `<a xmlns:xacro="http://www.ros.org/wiki/xacro">
                <xacro:macro name="bar">bar</xacro:macro>
                <xacro:property name="val" value="2" />
                <xacro:property name="some_block">
                    <some_block attr="\${val}"><xacro:bar/></some_block>
                </xacro:property>
                <foo>
                    <xacro:insert_block name="some_block" />
                </foo>
            </a>`,
            res => {
                expect(res).toEqual(unformat(`
                    <a>
                        <foo><some_block attr="2">bar</some_block></foo>
                    </a>
                `));
            },
        ),
    );

    it.todo('test_include');
    it.todo('test_include_glob');
    it.todo('test_include_nonexistent');
    it.todo('test_include_deprecated');
    it.todo('test_include_from_variable');
    it.todo('test_include_recursive');
    it.todo('test_include_with_namespace');

    it('test_boolean_if_statement', async() =>
        runXacro(
            `<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
                <xacro:if value="false">
                    <a/>
                </xacro:if>
                <xacro:if value="true">
                    <b/>
                </xacro:if>
            </robot>`,
            res => {
                expect(res).toEqual(unformat(`
                    <robot>
                        <b/>
                    </robot>
                `));
            },
        ),
    );

    it.todo('test_invalid_if_statement');

    it('test_integer_if_statement', async() =>
        runXacro(
            `<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
                <xacro:if value="\${0*42}">
                    <a />
                </xacro:if>
                <xacro:if value="0">
                    <b />
                </xacro:if>
                <xacro:if value="\${0}">
                    <c />
                </xacro:if>
                <xacro:if value="\${1*2+3}">
                    <d />
                </xacro:if>
            </robot>`,
            res => {
                expect(res).toEqual(unformat(`
                    <robot>
                        <d/>
                    </robot>
                `));
            },
        ),
    );

    it('test_float_if_statement', async() =>
        runXacro(
            `<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
                <xacro:if value="\${3*0.0}">
                    <a />
                </xacro:if>
                <xacro:if value="\${3*0.1}">
                    <b />
                </xacro:if>
            </robot>`,
            res => {
                expect(res).toEqual(unformat(`
                    <robot>
                        <b/>
                    </robot>
                `));
            },
        ),
    );

    it('test_property_if_statement', async() =>
        runXacro(
            `<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
                <xacro:property name="condT" value="\${True}"/>
                <xacro:property name="condF" value="\${False}"/>
                <xacro:if value="\${condF}"><a /></xacro:if>
                <xacro:if value="\${condT}"><b /></xacro:if>
                <xacro:if value="\${True}"><c /></xacro:if>
            </robot>`,
            res => {
                expect(res).toEqual(unformat(`
                    <robot>
                        <b/><c/>
                    </robot>
                `));
            },
        ),
    );

    it('test_consecutive_if', async() =>
        runXacro(
            `<a xmlns:xacro="http://www.ros.org/wiki/xacro">
                <xacro:if value="1">
                    <xacro:if value="0">
                        <a>bar</a>
                    </xacro:if>
                </xacro:if>
            </a>`,
            res => {
                expect(res).toEqual(unformat(`
                    <a></a>
                `));
            },
        ),
    );

    it('test_equality_expression_in_if_statement', async() =>
        runXacro(
            `<a xmlns:xacro="http://www.ros.org/wiki/xacro">
                <xacro:property name="var" value="useit"/>
                <xacro:if value="\${var == 'useit'}">
                    <foo>bar</foo>
                </xacro:if>
                <xacro:if value="\${'use' in var}">
                    <bar>foo</bar>
                </xacro:if>
            </a>`,
            res => {
                expect(res).toEqual(unformat(`
                    <a>
                        <foo>bar</foo>
                        <bar>foo</bar>
                    </a>
                `));
            },
        ),
    );

    it('test_no_evaluation', async() =>
        runXacro(
            `<a xmlns:xacro="http://www.ros.org/wiki/xacro">
                <xacro:property name="xyz" value="5 -2"/>
                <foo>\${xyz}</foo>
            </a>`,
            res => {
                expect(res).toEqual(unformat(`
                    <a>
                        <foo>5 -2</foo>
                    </a>
                `));
            },
        ),
    );

    it('test_math_expressions', async() =>
        runXacro(
            `<a xmlns:xacro="http://www.ros.org/wiki/xacro">
                <foo function="\${1. + sin(pi)}"/>
            </a>`,
            res => {
                expect(res).toEqual(unformat(`
                    <a>
                        <foo function="1.0"/>
                    </a>
                `));
            },
        ),
    );

    it('test_consider_non_elements_if', async() =>
        runXacro(
            `<a xmlns:xacro="http://www.ros.org/wiki/xacro">
                <xacro:if value="1"><!-- comment --> text <b>bar</b></xacro:if>
            </a>`,
            res => {
                expect(res).toEqual(unformat(`
                    <a>
                        <!-- comment --> text <b>bar</b>
                    </a>
                `));
            },
        ),
    );

    it('test_consider_non_elements_block', async() =>
        runXacro(
            `<a xmlns:xacro="http://www.ros.org/wiki/xacro">
                <xacro:macro name="foo" params="*block">
                    <!-- comment -->
                    foo
                    <xacro:insert_block name="block" />
                </xacro:macro>
                <xacro:foo>
                    <!-- ignored comment -->
                    ignored text
                    <a_block />
                </xacro:foo>
            </a>`,
            res => {
                expect(res).toEqual(unformat(`
                <a>
                    <!-- comment -->
                    foo
                    <a_block/>
                </a>
                `));
            },
        ),
    );

    it.skip('test_ignore_xacro_comments', async() =>
        runXacro(
            `<a xmlns:xacro="http://www.ros.org/wiki/xacro">
                <!-- A -->

                <!-- ignore multiline comments before any xacro tag -->
                <!-- ignored -->
                <xacro:property name="foo" value="1"/>
                <!-- ignored -->
                <xacro:if value="1"><!-- B --></xacro:if>
                <!-- ignored -->
                <xacro:macro name="foo"><!-- C --></xacro:macro>
                <!-- ignored -->
                <xacro:foo/>
            </a>`,
            res => {
                expect(res).toEqual(unformat(`
                <a>
                    <!-- A -->
                    <!-- B -->
                    <!-- C -->
                </a>
            `));
            },
        ),
    );

    it('test_recursive_evaluation', async() =>
        runXacro(
            `<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
                <xacro:property name="a" value=" 42 "/>
                <xacro:property name="a2" value="\${ 2 * a }"/>
                <a doubled="\${a2}"/>
            </robot>`,
            res => {
                expect(res).toEqual(unformat(`
                <robot>
                    <a doubled="84"/>
                </robot>
            `));
            },
        ),
    );

    it('test_recursive_evaluation', async() =>
        runXacro(
            `<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
                <xacro:property name="a2" value="\${2*a}"/>
                <xacro:property name="a" value="42"/>
                <a doubled="\${a2}"/>
            </robot>`,
            res => {
                expect(res).toEqual(unformat(`
                <robot>
                    <a doubled="84"/>
                </robot>
            `));
            },
        ),
    );

    it('test_recursive_definition', done => {
        runXacro(
            `<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
                <xacro:property name="a" value="\${a2}"/>
                <xacro:property name="a2" value="\${2*a}"/>
                <a doubled="\${a2}"/>
            </robot>`,
            () => done(new Error()),
            null,
            () => done(),
        );
    },
    );

    it('test_multiple_recursive_evaluation', async() =>
        runXacro(
            `<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
                <xacro:property name="a" value="1"/>
                <xacro:property name="b" value="2"/>
                <xacro:property name="c" value="3"/>
                <xacro:property name="product" value="\${a*b*c}"/>
                <answer product="\${product}"/>
            </robot>`,
            res => {
                expect(res).toEqual(unformat(`
                <robot>
                    <answer product="6"/>
                </robot>
            `));
            },
        ),
    );

    it('test_multiple_definition_and_evaluation', async() =>
        runXacro(
            `<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
                <xacro:property name="a" value="42"/>
                <xacro:property name="b" value="\${a}"/>
                <xacro:property name="b" value="\${-a}"/>
                <xacro:property name="b" value="\${a}"/>
                <answer b="\${b} \${b} \${b}"/>
            </robot>`,
            res => {
                expect(res).toEqual(unformat(`
                <robot>
                    <answer b="42 42 42"/>
                </robot>
            `));
            },
        ),
    );

    it('test_transitive_evaluation', async() =>
        runXacro(
            `<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
                <xacro:property name="a" value="42"/>
                <xacro:property name="b" value="\${a}"/>
                <xacro:property name="c" value="\${b}"/>
                <xacro:property name="d" value="\${c}"/>
                <answer d="\${d}"/>
            </robot>`,
            res => {
                expect(res).toEqual(unformat(`
                <robot>
                    <answer d="42"/>
                </robot>
            `));
            },
        ),
    );

    it('test_multi_tree_evaluation', async() =>
        runXacro(
            `<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
                <xacro:property name="a" value="42"/>
                <xacro:property name="b" value="2.1"/>
                <xacro:property name="c" value="\${a}"/>
                <xacro:property name="d" value="\${b}"/>
                <xacro:property name="f" value="\${c*d}"/>
                <answer f="\${f}"/>
            </robot>`,
            res => {
                expect(res).toEqual(unformat(`
                <robot>
                    <answer f="88.2"/>
                </robot>
            `));
            },
        ),
    );

    it('test_from_issue', async() =>
        runXacro(
            `<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
                <xacro:property name="x" value="42"/>
                <xacro:property name="wheel_width" value="\${x}"/>
                <link name="my_link">
                    <origin xyz="0 0 \${wheel_width/2}"/>
                </link>
            </robot>`,
            res => {
                expect(res).toEqual(unformat(`
                <robot>
                    <link name="my_link">
                        <origin xyz="0 0 21"/>
                    </link>
                </robot>
            `));
            },
        ),
    );

    it.todo('test_recursive_bad_math');

    // TODO: What's the difference between "=" and ":="
    it('test_default_param', async() =>
        runXacro(
            `<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
                <xacro:macro name="fixed_link" params="parent_link:=base_link child_link *joint_pose">
                    <link name="\${child_link}"/>
                    <joint name="\${child_link}_joint" type="fixed">
                        <xacro:insert_block name="joint_pose" />
                        <parent link="\${parent_link}"/>
                        <child link="\${child_link}" />
                    </joint>
                </xacro:macro>
                <xacro:fixed_link child_link="foo">
                    <origin xyz="0 0 0" rpy="0 0 0" />
                </xacro:fixed_link >
            </robot>`,
            res => {
                expect(res).toEqual(unformat(`
                <robot>
                    <link name="foo"/>
                    <joint name="foo_joint" type="fixed">
                        <origin xyz="0 0 0" rpy="0 0 0"/>
                        <parent link="base_link"/>
                        <child link="foo"/>
                    </joint>
                </robot>
            `));
            },
        ),
    );

    it('test_default_param_override', async() =>
        runXacro(
            `<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
                <xacro:macro name="fixed_link" params="parent_link:=base_link child_link *joint_pose">
                    <link name="\${child_link}"/>
                    <joint name="\${child_link}_joint" type="fixed">
                        <xacro:insert_block name="joint_pose" />
                        <parent link="\${parent_link}"/>
                        <child link="\${child_link}" />
                    </joint>
                </xacro:macro>
                <xacro:fixed_link child_link="foo" parent_link="bar">
                    <origin xyz="0 0 0" rpy="0 0 0" />
                </xacro:fixed_link >
            </robot>`,
            res => {
                expect(res).toEqual(unformat(`
                <robot>
                    <link name="foo"/>
                    <joint name="foo_joint" type="fixed">
                        <origin xyz="0 0 0" rpy="0 0 0"/>
                        <parent link="bar"/>
                        <child link="foo"/>
                    </joint>
                </robot>
            `));
            },
        ),
    );

    it.todo('test_param_missing');

    it('test_default_arg', async() =>
        runXacro(`<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
                    <xacro:arg name="foo" default="2"/>
                    <link name="my_link">
                        <origin xyz="0 0 $(arg foo)"/>
                    </link>
                </robot>`,
        res => {
            expect(res).toEqual(unformat(`
            <robot>
                <link name="my_link">
                    <origin xyz="0 0 2"/>
                </link>
            </robot>
        `));
        }),
    );

    it('test_default_arg_override', async() =>
        runXacro(`<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
                    <xacro:arg name="foo" default="2"/>
                    <link name="my_link">
                        <origin xyz="0 0 $(arg foo)"/>
                    </link>
                </robot>`,
        res => {
            expect(res).toEqual(unformat(
                `<robot>
                    <link name="my_link">
                        <origin xyz="0 0 4"/>
                    </link>
                </robot>`));
        },
        {arguments: {foo: 4}},
        ),
    );

    it('test_default_arg_missing', (done) => {
        runXacro(`<a xmlns:xacro="http://www.ros.org/wiki/xacro">
                    <a arg="$(arg foo)"/>
                </a>`,
        () => done(new Error()),
        null,
        () => done());
    });

    it('test_default_arg_empty', async() =>
        runXacro(`<a xmlns:xacro="http://www.ros.org/wiki/xacro">
    <xacro:arg name="foo" default=""/>$(arg foo)</a>`,
        res => { expect(res).toEqual(unformat(`<a></a>`)); },
        ),
    );

    it.todo('test_broken_input_doesnt_create_empty_output_file');
    it.todo('test_create_subdirs');
    it.todo('test_iterable_literals_plain');
    it.todo('test_iterable_literals_eval');
    it.todo('test_enforce_xacro_ns');

    it('test_issue_68_numeric_arg', async() =>
        runXacro(
            `<a xmlns:xacro="http://www.ros.org/wiki/xacro">
                <xacro:arg name="foo" default="0.5"/>
                <xacro:property name="prop" value="$(arg foo)" />
                <a prop="\${prop-0.3}"/>
            </a>`,
            res => {
                expect(res).toEqual(unformat(
                    `<a>
                        <a prop="0.2"/>
                    </a>`));
            }),
    );

    it('test_transitive_arg_evaluation', async() =>
        runXacro(`<a xmlns:xacro="http://www.ros.org/wiki/xacro">
                    <xacro:arg name="foo" default="0.5"/>
                    <xacro:arg name="bar" default="$(arg foo)"/>
                    <xacro:property name="prop" value="$(arg bar)" />
                    <a prop="\${prop-0.3}"/>
                </a>`, res => {
            expect(res).toEqual(unformat(
                `<a>
                    <a prop="0.2"/>
                </a>`));
        }),
    );

    it('test_macro_name_with_colon', async() =>
        runXacro(
            `<a xmlns:xacro="http://www.ros.org/wiki/xacro">
                <xacro:macro name="xacro:my_macro">
                    <foo/>
                </xacro:macro>
                <xacro:my_macro/>
            </a>`,
            res => {
                expect(res).toEqual(unformat(`
                <a><foo/></a>
            `));
            },
        ),
    );

    it.todo('test_overwrite_globals');

    it('test_no_double_evaluation', async() =>
        runXacro(
            `<a xmlns:xacro="http://www.ros.org/xacro">
                <xacro:macro name="foo" params="a b:=\${a} c:=$\${a}"> a=\${a} b=\${b} c=\${c} </xacro:macro>
                <xacro:property name="a" value="1"/>
                <xacro:property name="d" value="$\${a}"/>
                <d d="\${d}"><foo a="2"/></d>
            </a>`,
            res => {
                expect(res).toEqual(unformat(`
                <a><d d="\${a}"> a=2 b=1 c=\${a} </d></a>
            `));
            },
        ),
    );

    it.todo('test_property_forwarding');

    it('test_extension_in_expression', async() =>
        runXacro(
            `<a xmlns:xacro="http://www.ros.org/wiki/xacro">\${'test'+'$(arg var)'}</a>`,
            res => {
                expect(res).toEqual(unformat(`
                <a>testxacro</a>
            `));
            },
            {
                arguments: {var: 'xacro'},
            },
        ),
    );

    it('test_extension_in_expression', async() =>
        runXacro(
            `<a xmlns:xacro="http://www.ros.org/wiki/xacro">$(arg \${'v'+'ar'})</a>`,
            res => {
                expect(res).toEqual(unformat(`
                <a>xacro</a>
            `));
            },
            {
                arguments: {var: 'xacro'},
            },
        ),
    );

    it.todo('test_target_namespace');
    it.todo('test_target_namespace_only_from_root');

});

describe('In Order Xacro Tests', () => {
    it('test_include_lazy', async() =>
        runXacro(
            `<a xmlns:xacro="http://www.ros.org/xacro">
                <xacro:if value="false">
                    <xacro:include filename="non-existent"/>
                </xacro:if>
            </a>`,
            res => {
                expect(res).toEqual(unformat(`
                <a></a>
            `));
            },
        ),
    );

    it('test_issue_63_fixed_with_inorder_processing', async() =>
        runXacro(
            `<a xmlns:xacro="http://www.ros.org/wiki/xacro">
                <xacro:arg name="has_stuff" default="false"/>
                <xacro:if value="$(arg has_stuff)">
                    <xacro:include file="$(find nonexistent_package)/stuff.urdf" />
                </xacro:if>
            </a>`,
            res => {
                expect(res).toEqual(unformat(`
                <a></a>
            `));
            },
        ),
    );

    it.todo('test_yaml_support');

    it('test_macro_default_param_evaluation_order', async() =>
        runXacro(
            `<a xmlns:xacro="http://www.ros.org/wiki/xacro">
                <xacro:macro name="foo" params="arg:=\${2*foo}">
                    <xacro:property name="foo" value="-"/>
                    <f val="\${arg}"/>
                </xacro:macro>
                <xacro:property name="foo" value="\${3*7}"/>
                <xacro:foo/>
                <xacro:property name="foo" value="100"/>
                <xacro:foo/>
            </a>`,
            res => {
                expect(res).toEqual(unformat(`
                <a><f val="42"/><f val="200"/></a>
            `));
            },
        ),
    );

    it.todo('test_check_order_warning');

    it('test_default_property', async() =>
        runXacro(
            `<a xmlns:xacro="http://www.ros.org/xacro">
                <xacro:property name="prop" default="false"/>
                <xacro:unless value="\${prop}">
                    <foo/>
                    <xacro:property name="prop" value="true"/>
                </xacro:unless>

                <!-- second foo should be ignored -->
                <xacro:unless value="\${prop}">
                    <foo/>
                    <xacro:property name="prop" value="true"/>
                </xacro:unless>
            </a>`,
            res => {
                expect(res).toEqual(unformat(`
                <a><foo/></a>
            `));
            },
        ),
    );

    it('test_unicode_literal_parsing', async() =>
        runXacro(
            `<a xmlns:xacro="http://www.ros.org/wiki/xacro">🍔 </a>`,
            res => {
                expect(res).toEqual(unformat(`
                <a>🍔 </a>
            `));
            },
        ),
    );

    it('test_unicode_property', async() =>
        runXacro(
            `<a xmlns:xacro="http://www.ros.org/wiki/xacro">
                <xacro:property name="burger" value="🍔"/>
                <b c="\${burger}"/>
            </a>`,
            res => {
                expect(res).toEqual(unformat(`
                <a><b c="🍔"/></a>
            `));
            },
        ),
    );

    it.todo('test_unicode_property_attribute');
    it.todo('test_unicode_property_block');
    it.todo('test_unicode_conditional');
    it.todo('test_unicode_macro');
    it.todo('test_unicode_file');

});
