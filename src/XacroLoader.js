import { getUrlBase } from './utils.js';
import { XacroParser } from './XacroParser.js';

/**
 * @callback XacroLoaderCompleteCallback
 * @param {XMLDocument} result
 */

/**
 * @callback XacroLoaderErrorCallback
 * @param {Error} error
 */

/**
 * Extends XacroParser and implements `getFileContents` to load from a server using fetch.
 *
 * ```js
 * const loader = new XacroLoader();
 * loader.load(
 *     '../path/to/file.xacro',
 *     result => {
 *
 *         // xacro XML
 *
 *     },
 *     err => {
 *
 *         // parse error
 *
 *     } );
 * ```
 * @note The working path is extracted automatically from the loaded url. Only works in the browser.
 * @extends XacroParser
 */
export class XacroLoader extends XacroParser {

	constructor() {

		super();

		/**
		 * Options passed to `fetch` when loading files.
		 * @type {RequestInit}
		 * @default {}
		 */
		this.fetchOptions = {};

	}

	/**
	 * Loads and parses the xacro file at the given url. If `workingPath` has not been set it is
	 * extracted from the url.
	 * @param {string} url
	 * @param {XacroLoaderCompleteCallback} onComplete
	 * @param {XacroLoaderErrorCallback} [onError]
	 * @returns {void}
	 */
	load( url, onComplete, onError ) {

		const workingPath = getUrlBase( url );
		if ( this.workingPath === '' ) {

			this.workingPath = workingPath;

		}

		this
			.getFileContents( url )
			.then( text => {

				this.parse( text, onComplete, onError );

			} )
			.catch( e => {

				if ( onError ) {

					onError( e );

				}

			} );

	}

	/**
	 * Parses the passed xacro contents using the options specified on the object and calls
	 * `onComplete` with the xml document of the processed xacro file.
	 * @param {string} data
	 * @param {XacroLoaderCompleteCallback} onComplete
	 * @param {XacroLoaderErrorCallback} [onError]
	 * @returns {void}
	 */
	parse( data, onComplete, onError ) {

		super
			.parse( data )
			.then( onComplete )
			.catch( e => {

				if ( onError ) {

					onError( e );

				}

			} );

	}

	/**
	 * Loads the file at the given path using `fetch` and `fetchOptions` and returns the contents
	 * as a string.
	 * @param {string} path
	 * @returns {Promise<string>}
	 */
	getFileContents( path ) {

		return fetch( path, this.fetchOptions )
			.then( res => {

				if ( res.ok ) {

					return res.text();

				} else {

					throw new Error( `XacroLoader: Failed to load url '${ path }' with error code ${ res.status } : ${ res.statusText }.` );

				}

			} );

	}

}
