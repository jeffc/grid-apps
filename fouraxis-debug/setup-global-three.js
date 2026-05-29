import * as THREE from './src/ext/three.js';
import './src/add/array.js';
global.THREE = THREE.THREE || THREE;
global.self = global;
global.self.ClipperLib = {}; // Mock ClipperLib to avoid reference errors if any
