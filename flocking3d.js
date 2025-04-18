// flocking3d.js
import * as THREE from 'https://esm.sh/three@0.152.2';
import {GPUComputationRenderer} from 'https://esm.sh/three@0.152.2/examples/jsm/misc/GPUComputationRenderer.js';

class TriBoidGeometry extends THREE.BufferGeometry {
    /**
     * Builds N boids, each as a single triangle in the XY‑plane.
     * You can swap this out for lines, ovals, etc.
     */
    constructor(nBoids) {
        super();
        const vertsPerBoid = 3;
        const posArray = new Float32Array(nBoids * vertsPerBoid * 3);
        const refArray = new Float32Array(nBoids * vertsPerBoid * 2);
        let vIdx = 0, rIdx = 0;

        for (let i = 0; i < nBoids; i++) {
            // equilateral triangle of side ~1, centered at origin
            const size = 1;
            const angles = [0, (2 * Math.PI / 3), (4 * Math.PI / 3)];
            for (const a of angles) {
                posArray[vIdx++] = Math.cos(a) * size;
                posArray[vIdx++] = Math.sin(a) * size;
                posArray[vIdx++] = 0;
                // reference into the computation texture
                const u = (i % texSize) / texSize;
                const v = Math.floor(i / texSize) / texSize;
                refArray[rIdx++] = u;
                refArray[rIdx++] = v;
            }
        }

        this.setAttribute('position', new THREE.BufferAttribute(posArray, 3));
        this.setAttribute('reference', new THREE.BufferAttribute(refArray, 2));
    }
}

export default class FlockingSimulator {
    constructor(canvas, opts = {}) {
        // Canvas and renderer
        this.canvas = (typeof canvas === 'string')
            ? document.getElementById(canvas)
            : canvas;
        if (!(this.canvas instanceof HTMLCanvasElement))
            throw new Error('Canvas or ID required');

        this.renderer = new THREE.WebGLRenderer({canvas: this.canvas, antialias: true});
        this.renderer.setSize(this.canvas.clientWidth, this.canvas.clientHeight);

        // Configuration
        const defaults = {
            boidCount: 256,        // must be a square number for the GPU texture
            bounds: 500,
            // Boid behavior distances (in world units)
            separation: 20,
            alignment: 30,
            cohesion: 30,
            freedom: 0.3,
            // Depth cueing
            minSize: 0.5,
            maxSize: 2.5,
            nearColor: 0x222222,
            farColor: 0x888888,
            background: 0x87ceeb,
            // camera
            fov: 75,
            nearClip: 1,
            farClip: 3000
        };
        this.cfg = {...defaults, ...opts};

        // Three.js scene
        const {boidCount, bounds, background, fov, nearClip, farClip} = this.cfg;
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(background);
        this.camera = new THREE.PerspectiveCamera(
            fov, this.canvas.clientWidth / this.canvas.clientHeight, nearClip, farClip
        );
        this.camera.position.set(bounds, bounds, bounds);
        this.camera.lookAt(0, 0, 0);

        // GPUComputationRenderer setup
        this.texSize = Math.sqrt(boidCount) | 0;
        this.gpu = new GPUComputationRenderer(this.texSize, this.texSize, this.renderer);
        const posTex = this.gpu.createTexture();
        const velTex = this.gpu.createTexture();
        this._fillTextures(posTex, velTex);

        this.velVar = this.gpu.addVariable(
            'VeloctiyTexture', document.getElementById('BoidVelocityFragmentShader').textContent, velTex
        );
        this.posVar = this.gpu.addVariable(
            'PositionTexture', document.getElementById('BoidPositionFragmentShader').textContent, posTex
        );
        this.gpu.setVariableDependencies(this.velVar, [this.posVar, this.velVar]);
        this.gpu.setVariableDependencies(this.posVar, [this.posVar, this.velVar]);

        // uniforms
        this.velVar.material.uniforms = {
            separation: {value: this.cfg.separation},
            alignment: {value: this.cfg.alignment},
            cohesion: {value: this.cfg.cohesion},
            freedom: {value: this.cfg.freedom},
            bounds: {value: bounds}
        };
        this.posVar.material.uniforms = {
            bounds: {value: bounds}
        };
        this.gpu.init();

        // Build the mesh
        const geom = new TriBoidGeometry(boidCount);
        const material = new THREE.ShaderMaterial({
            uniforms: {
                PositionTexture: {value: null},
                VeloctiyTexture: {value: null},
                minSize: {value: this.cfg.minSize},
                maxSize: {value: this.cfg.maxSize},
                nearColor: {value: new THREE.Color(this.cfg.nearColor)},
                farColor: {value: new THREE.Color(this.cfg.farColor)},
                bounds: {value: bounds}
            },
            vertexShader: document.getElementById('BoidVertexShader').textContent,
            fragmentShader: document.getElementById('BoidGeometryFragmentShader').textContent,
            depthTest: true,
            transparent: false
        });
        this.boidMesh = new THREE.Mesh(geom, material);
        this.boidMesh.matrixAutoUpdate = false;
        this.scene.add(this.boidMesh);

        // animation
        this.clock = new THREE.Clock();
        this.running = false;
        this._loop = this._loop.bind(this);
    }

    _fillTextures(posTex, velTex) {
        const arrP = posTex.image.data;
        const arrV = velTex.image.data;
        for (let i = 0; i < arrP.length; i += 4) {
            // random in [-B/2, B/2]
            arrP[i] = (Math.random() - 0.5) * this.cfg.bounds;
            arrP[i + 1] = (Math.random() - 0.5) * this.cfg.bounds;
            arrP[i + 2] = (Math.random() - 0.5) * this.cfg.bounds;
            arrP[i + 3] = 1;
            // random velocity
            arrV[i] = (Math.random() - 0.5) * 10;
            arrV[i + 1] = (Math.random() - 0.5) * 10;
            arrV[i + 2] = (Math.random() - 0.5) * 10;
            arrV[i + 3] = 1;
        }
    }

    start() {
        if (!this.running) {
            this.running = true;
            this.clock.start();
            this._loop();
        }
    }

    stop() {
        this.running = false;
    }

    _loop() {
        if (!this.running) return;
        const dt = this.clock.getDelta();

        // step GPU simulation
        this.velVar.material.uniforms.clock = {value: performance.now()};
        this.velVar.material.uniforms.del_change = {value: dt};
        this.posVar.material.uniforms.clock = {value: performance.now()};
        this.posVar.material.uniforms.del_change = {value: dt};
        this.gpu.compute();

        // update mesh uniforms
        this.boidMesh.material.uniforms.PositionTexture.value = this.gpu.getCurrentRenderTarget(this.posVar).texture;
        this.boidMesh.material.uniforms.VeloctiyTexture.value = this.gpu.getCurrentRenderTarget(this.velVar).texture;

        // render
        this.renderer.render(this.scene, this.camera);

        requestAnimationFrame(this._loop);
    }
}