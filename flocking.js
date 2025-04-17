// flocking.js
export default class FlockingSimulator {
    constructor(canvas, opts = {}) {
        // Canvas setup
        this.canvas =
            typeof canvas === 'string'
                ? document.getElementById(canvas)
                : canvas;
        if (!(this.canvas instanceof HTMLCanvasElement)) {
            throw new Error('Canvas element or ID required');
        }
        this.ctx = this.canvas.getContext('2d');
        this.width = this.canvas.width;
        this.height = this.canvas.height;

        // Configuration with sensible defaults
        const defaults = {
            boidCount: 200,
            maxSpeed: 2.0,
            maxForce: 0.03,
            perceptionRadius: 50,
            separationRadius: 25,
            alignmentWeight: 1.0,
            cohesionWeight: 1.0,
            separationWeight: 1.8,
            backgroundColor: '#ffffff',
            boidColor: '#333333',
            boidSize: 5,

            // New: moving disturbance (hazard) points
            disturbanceCount: 3,    // number of moving hazards
            disturbanceMinSpeed: 0.2,    // min speed for hazard points
            disturbanceMaxSpeed: 1.0,    // max speed for hazard points
            disturbanceRadius: 125,     // how close boids will start to avoid
            disturbanceRepulsionStrength: 1.75,  // repulsion force multiplier
        };
        this.cfg = {...defaults, ...opts};

        // State
        this.boids = [];
        this.disturbancePoints = [];
        this.running = false;
        this._animate = this._animate.bind(this);

        this._initBoids();
        this._initDisturbancePoints();
    }

    /**
     * Populate initial boids
     */
    _initBoids() {
        for (let i = 0; i < this.cfg.boidCount; i++) {
            this.boids.push({
                x: Math.random() * this.width,
                y: Math.random() * this.height,
                vx: (Math.random() * 2 - 1) * this.cfg.maxSpeed,
                vy: (Math.random() * 2 - 1) * this.cfg.maxSpeed,
                ax: 0,
                ay: 0,
            });
        }
    }

    /**
     * Create initial moving disturbance points ("hazards") that boids will avoid.
     */
    _initDisturbancePoints() {
        const {
            disturbanceCount,
            disturbanceMinSpeed,
            disturbanceMaxSpeed,
        } = this.cfg;
        this.disturbancePoints = [];
        for (let i = 0; i < disturbanceCount; i++) {
            const x = Math.random() * this.width;
            const y = Math.random() * this.height;
            const angle = Math.random() * 2 * Math.PI;
            const speed =
                disturbanceMinSpeed +
                Math.random() * (disturbanceMaxSpeed - disturbanceMinSpeed);
            this.disturbancePoints.push({
                x,
                y,
                vx: Math.cos(angle) * speed,
                vy: Math.sin(angle) * speed,
            });
        }
    }

    /**
     * Apply a slight random steering to each disturbance point and wrap around edges.
     */
    _updateDisturbancePoints() {
        const {
            disturbanceMinSpeed,
            disturbanceMaxSpeed,
        } = this.cfg;
        for (const p of this.disturbancePoints) {
            // small random angle jitter
            const baseAngle = Math.atan2(p.vy, p.vx);
            const angle = baseAngle + (Math.random() - 0.5) * 0.2;
            // small random speed tweak
            let speed = Math.hypot(p.vx, p.vy) + (Math.random() - 0.5) * 0.1;
            speed = Math.max(disturbanceMinSpeed, Math.min(disturbanceMaxSpeed, speed));
            p.vx = Math.cos(angle) * speed;
            p.vy = Math.sin(angle) * speed;
            // move and wrap
            p.x = (p.x + p.vx + this.width) % this.width;
            p.y = (p.y + p.vy + this.height) % this.height;
        }
    }

    // Public API --------------------------------------------------

    start() {
        if (!this.running) {
            this.running = true;
            requestAnimationFrame(this._animate);
        }
    }

    stop() {
        this.running = false;
    }

    // Main loop ----------------------------------------------------

    _animate() {
        if (!this.running) return;
        this._update();
        this._draw();
        requestAnimationFrame(this._animate);
    }

    _update() {
        const {
            perceptionRadius,
            separationRadius,
            maxSpeed,
            maxForce,
            alignmentWeight,
            cohesionWeight,
            separationWeight,
            disturbanceRadius,
            disturbanceRepulsionStrength,
        } = this.cfg;

        // Move the disturbance points first
        this._updateDisturbancePoints();

        // Spatial hash for boid–boid neighbor lookup
        const cellSize = perceptionRadius;
        const cols = Math.ceil(this.width / cellSize);
        const rows = Math.ceil(this.height / cellSize);
        const grid = {};
        for (const b of this.boids) {
            const col = Math.floor(b.x / cellSize);
            const row = Math.floor(b.y / cellSize);
            const key = col + ',' + row;
            (grid[key] ||= []).push(b);
        }

        // For each boid, compute flocking forces + hazard avoidance
        for (const b of this.boids) {
            let ax = 0, ay = 0;
            let totalA = 0, avgVx = 0, avgVy = 0;
            let totalC = 0, avgCx = 0, avgCy = 0;
            let totalS = 0, sepX = 0, sepY = 0;

            // 1) Alignment, cohesion, separation
            const bc = Math.floor(b.x / cellSize);
            const br = Math.floor(b.y / cellSize);
            for (let i = bc - 1; i <= bc + 1; i++) {
                for (let j = br - 1; j <= br + 1; j++) {
                    for (const other of grid[i + ',' + j] || []) {
                        if (other === b) continue;
                        const dx = other.x - b.x;
                        const dy = other.y - b.y;
                        const dist = Math.hypot(dx, dy);
                        if (dist < perceptionRadius) {
                            // alignment
                            avgVx += other.vx;
                            avgVy += other.vy;
                            totalA++;
                            // cohesion
                            avgCx += other.x;
                            avgCy += other.y;
                            totalC++;
                            // separation
                            if (dist < separationRadius) {
                                sepX += b.x - other.x;
                                sepY += b.y - other.y;
                                totalS++;
                            }
                        }
                    }
                }
            }

            // Steering computations...
            if (totalA > 0) {
                avgVx /= totalA;
                avgVy /= totalA;
                let mag = Math.hypot(avgVx, avgVy) || 1;
                avgVx = (avgVx / mag) * maxSpeed - b.vx;
                avgVy = (avgVy / mag) * maxSpeed - b.vy;
                mag = Math.hypot(avgVx, avgVy) || 1;
                ax += (avgVx / mag) * Math.min(mag, maxForce) * alignmentWeight;
                ay += (avgVy / mag) * Math.min(mag, maxForce) * alignmentWeight;
            }
            if (totalC > 0) {
                avgCx /= totalC;
                avgCy /= totalC;
                let vx = avgCx - b.x, vy = avgCy - b.y;
                let mag = Math.hypot(vx, vy) || 1;
                vx = (vx / mag) * maxSpeed - b.vx;
                vy = (vy / mag) * maxSpeed - b.vy;
                mag = Math.hypot(vx, vy) || 1;
                ax += (vx / mag) * Math.min(mag, maxForce) * cohesionWeight;
                ay += (vy / mag) * Math.min(mag, maxForce) * cohesionWeight;
            }
            if (totalS > 0) {
                sepX /= totalS;
                sepY /= totalS;
                let mag = Math.hypot(sepX, sepY) || 1;
                sepX = (sepX / mag) * maxSpeed - b.vx;
                sepY = (sepY / mag) * maxSpeed - b.vy;
                mag = Math.hypot(sepX, sepY) || 1;
                ax += (sepX / mag) * Math.min(mag, maxForce) * separationWeight;
                ay += (sepY / mag) * Math.min(mag, maxForce) * separationWeight;
            }

            // 2) Avoid disturbance points
            for (const p of this.disturbancePoints) {
                const dx = b.x - p.x;
                const dy = b.y - p.y;
                const dist = Math.hypot(dx, dy);
                if (dist < disturbanceRadius) {
                    const force = disturbanceRepulsionStrength * (1 - dist / disturbanceRadius);
                    ax += (dx / dist) * force;
                    ay += (dy / dist) * force;
                }
            }

            // Apply acceleration
            b.ax = ax;
            b.ay = ay;
        }

        // Motion & wrapping for boids
        for (const b of this.boids) {
            b.vx += b.ax;
            b.vy += b.ay;
            const speed = Math.hypot(b.vx, b.vy) || 1;
            if (speed > maxSpeed) {
                b.vx = (b.vx / speed) * maxSpeed;
                b.vy = (b.vy / speed) * maxSpeed;
            }
            b.x = (b.x + b.vx + this.width) % this.width;
            b.y = (b.y + b.vy + this.height) % this.height;
        }
    }

    _draw() {
        const {backgroundColor, boidColor, boidSize} = this.cfg;
        const ctx = this.ctx;
        // background
        ctx.fillStyle = backgroundColor;
        ctx.fillRect(0, 0, this.width, this.height);

        // draw hazard points (optional—for debugging you can uncomment)
        // ctx.fillStyle = 'rgba(255,0,0,0.6)';
        // for (const p of this.disturbancePoints) {
        //   ctx.beginPath();
        //   ctx.arc(p.x, p.y, 5, 0, 2*Math.PI);
        //   ctx.fill();
        // }

        // draw boids
        ctx.fillStyle = boidColor;
        for (const b of this.boids) {
            const angle = Math.atan2(b.vy, b.vx);
            ctx.save();
            ctx.translate(b.x, b.y);
            ctx.rotate(angle);
            ctx.beginPath();
            ctx.moveTo(boidSize, 0);
            ctx.lineTo(-boidSize * 0.5, boidSize * 0.5);
            ctx.lineTo(-boidSize * 0.5, -boidSize * 0.5);
            ctx.closePath();
            ctx.fill();
            ctx.restore();
        }
    }
}