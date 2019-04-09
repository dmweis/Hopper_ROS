/*****************************************************************************
 * Open MCT, Copyright (c) 2014-2017, United States Government
 * as represented by the Administrator of the National Aeronautics and Space
 * Administration. All rights reserved.
 *
 * Open MCT is licensed under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0.
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 * Open MCT includes source code licensed under additional open source
 * licenses. See the Open Source Licenses file (LICENSES.md) included with
 * this source code distribution or the Licensing information page available
 * at runtime from the About dialog for additional information.
 *****************************************************************************/


function ImageryPlugin() {

    var IMAGE_SAMPLES = [
        "http://localhost:8080/snapshot?topic=/camera/rgb/image_color"
    ];

    function pointForTimestamp(timestamp, name) {
        return {
            name: name,
            utc: Math.floor(timestamp / 5000) * 5000,
            url: IMAGE_SAMPLES[0]
        };
    }

    var imageProvider = {
        supportsSubscribe: function (domainObject) {
            console.log(domainObject)
            return domainObject.type === 'hopper.telemetry' && domainObject.name === "camera";
        },
        subscribe: function (domainObject, callback) {
            var interval = setInterval(function () {
                callback(pointForTimestamp(Date.now(), domainObject.name));
            }, 1000);

            return function () {
                clearInterval(interval);
            };
        }
    };




    return function install(openmct) {
        openmct.types.addType('hopper.imagery', {
            key: 'example.imagery',
            name: 'Example Imagery',
            cssClass: 'icon-image',
            description: 'For development use. Creates example imagery ' +
                'data that mimics a live imagery stream.'
        });

        openmct.telemetry.addProvider(imageProvider);
    };
}