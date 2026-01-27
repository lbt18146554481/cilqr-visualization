import http.server
import socketserver
import json
import os
import subprocess
import sys
from urllib.parse import urlparse, parse_qs

PORT = 8081
BASE_DIR = os.path.dirname(os.path.abspath(__file__))  # visual目录
EXECUTABLE_PATH = os.path.join(BASE_DIR, "build_x86_64_Release", "planning_test_execute")
CONFIG_PATH = os.path.join(BASE_DIR, "planning_config.json")  # 配置文件在visual目录下

DEFAULT_VALUES = {
    "init_x": 0.0,
    "init_y": 0.0,
    "init_theta": 0.0,
    "init_kappa": 0.0,
    "init_v": 10.0,
    "init_a": 0.0,
    "ref_mode": "straight",
    "cilqr_trajectory_conf": {
        "position_weight": 1.0,
        "heading_weight": 1.0,
        "kappa_weight": 4.0,
        "speed_weight": 1.0,
        "acc_weight": 1.0,
        "dkappa_weight": 2.0,
        "jerk_weight": 0.1
    }
}

HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <title>CILQR Trajectory Tuner</title>
    <style>
        body { font-family: sans-serif; margin: 10px; display: flex; }
        .controls { width: 260px; padding-right: 10px; border-right: 1px solid #ccc; }
        .visualization { flex-grow: 1; padding-left: 10px; }
        .slider-container { margin-bottom: 10px; }
        .slider-label { display: flex; justify-content: space-between; margin-bottom: 2px; font-size: 0.9em; }
        input[type=range] { width: 100%; }
        .plots-row { display: flex; flex-wrap: wrap; gap: 5px; }
        .plot-box { border: 1px solid #eee; width: 32%; height: 250px; }
    </style>
    <script src="https://cdn.plot.ly/plotly-2.27.0.min.js"></script>
</head>
<body>
    <div class="controls">
        <h2>Tuning Parameters</h2>
        
        <div class="slider-container">
            <div class="slider-label">
                <span>Reference Mode</span>
            </div>
            <select id="ref-mode" onchange="updateRefMode(this.value)" style="width: 100%; padding: 5px;">
                <option value="straight">Straight</option>
                <option value="left_turn">Left Turn</option>
                <option value="right_turn">Right Turn</option>
                <option value="u_turn">U-Turn</option>
                <option value="lane_change">Lane Change</option>
            </select>
        </div>

        <details open style="margin-bottom: 15px; border: 1px solid #ccc; padding: 10px;">
            <summary style="cursor: pointer; font-weight: bold; margin-bottom: 10px;">Initial State</summary>
            <div style="display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 10px;">
                <label>x: <input type="number" id="init_x" step="0.1" value="0.0" onchange="updateInitState()" style="width: 60px;"></label>
                <label>y: <input type="number" id="init_y" step="0.1" value="0.0" onchange="updateInitState()" style="width: 60px;"></label>
                <label>heading: <input type="number" id="init_theta" step="0.01" value="0.0" onchange="updateInitState()" style="width: 60px;"></label>
                <label>kappa: <input type="number" id="init_kappa" step="0.01" value="0.0" onchange="updateInitState()" style="width: 60px;"></label>
                <label>v: <input type="number" id="init_v" step="0.1" value="10.0" onchange="updateInitState()" style="width: 60px;"></label>
                <label>a: <input type="number" id="init_a" step="0.1" value="0.0" onchange="updateInitState()" style="width: 60px;"></label>
            </div>
        </details>

        <details open style="margin-bottom: 15px; border: 1px solid #ccc; padding: 10px;">
            <summary style="cursor: pointer; font-weight: bold; margin-bottom: 10px;">Weights</summary>
            <div id="sliders"></div>
        </details>

        <div id="status" style="margin-top: 20px; color: #666;">Ready</div>
    </div>
    <div class="visualization">
        <h2>Trajectory Visualization</h2>
        <div class="plots-row">
            <div id="plot" class="plot-box"></div>
            <div id="plot-kappa" class="plot-box"></div>
            <div id="plot-dkappa" class="plot-box"></div>
            <div id="plot-v" class="plot-box"></div>
            <div id="plot-a" class="plot-box"></div>
            <div id="plot-jerk" class="plot-box"></div>
            <div id="plot-lat-acc" class="plot-box"></div>
            <div id="plot-l" class="plot-box"></div>
            <div id="plot-heading" class="plot-box"></div>
        </div>
    </div>

    <script>
        const params = [
            {name: 'position_weight', min: 0, max: 10, step: 0.1},
            {name: 'heading_weight', min: 0, max: 10, step: 0.1},
            {name: 'kappa_weight', min: 0, max: 20, step: 0.1},
            {name: 'speed_weight', min: 0, max: 10, step: 0.1},
            {name: 'acc_weight', min: 0, max: 10, step: 0.1},
            {name: 'dkappa_weight', min: 0, max: 10, step: 0.1},
            {name: 'jerk_weight', min: 0, max: 0.5, step: 0.05}
        ];

        let currentConfig = {};
        let lastResult = null;
        let simulationTimeout = null;

        // 防抖函数：避免频繁触发优化
        function debounceSimulation() {
            if (simulationTimeout) {
                clearTimeout(simulationTimeout);
            }
            simulationTimeout = setTimeout(() => {
                runSimulation();
            }, 300); // 300ms 延迟
        }

        function createSliders(initialConfig, fullConfig) {
            const container = document.getElementById('sliders');
            container.innerHTML = '';
            
            // Set Reference Mode
            if (fullConfig && fullConfig.ref_mode) {
                document.getElementById('ref-mode').value = fullConfig.ref_mode;
                currentConfig.ref_mode = fullConfig.ref_mode;
            } else {
                currentConfig.ref_mode = "straight";
            }

            // Set Initial State
            const initFields = ['init_x', 'init_y', 'init_theta', 'init_kappa', 'init_v', 'init_a'];
            const defaults = {
                'init_x': 0.0, 'init_y': 0.0, 'init_theta': 0.0, 
                'init_kappa': 0.0, 'init_v': 10.0, 'init_a': 0.0
            };
            
            initFields.forEach(field => {
                let val = (fullConfig && fullConfig[field] !== undefined) ? fullConfig[field] : defaults[field];
                document.getElementById(field).value = val;
                currentConfig[field] = val;
            });

            params.forEach(p => {
                const div = document.createElement('div');
                div.className = 'slider-container';
                
                let val = initialConfig[p.name] !== undefined ? initialConfig[p.name] : 1.0;
                
                div.innerHTML = `
                    <div class="slider-label">
                        <span>${p.name}</span>
                        <span id="val-${p.name}">${val}</span>
                    </div>
                    <input type="range" id="input-${p.name}" 
                           min="${p.min}" max="${p.max}" step="${p.step}" value="${val}"
                           oninput="updateParam('${p.name}', this.value); debounceSimulation()"
                           onchange="runSimulation()">
                `;
                container.appendChild(div);
                currentConfig[p.name] = val;
            });
        }

        function updateInitState() {
            const initFields = ['init_x', 'init_y', 'init_theta', 'init_kappa', 'init_v', 'init_a'];
            initFields.forEach(field => {
                currentConfig[field] = parseFloat(document.getElementById(field).value);
            });
            runSimulation();
        }

        function updateParam(name, value) {
            document.getElementById(`val-${name}`).innerText = value;
            currentConfig[name] = parseFloat(value);
        }

        function updateRefMode(value) {
            currentConfig.ref_mode = value;
            runSimulation();
        }

        async function runSimulation() {
            const status = document.getElementById('status');
            status.innerText = "Running...";
            
            try {
                // 确保所有权重参数都被包含
                const weightParams = ['position_weight', 'heading_weight', 'kappa_weight', 
                                     'speed_weight', 'acc_weight', 'dkappa_weight', 
                                     'jerk_weight'];
                const configToSend = {...currentConfig};
                weightParams.forEach(param => {
                    const slider = document.getElementById(`input-${param}`);
                    if (slider) {
                        configToSend[param] = parseFloat(slider.value);
                    }
                });
                
                console.log("Sending request with config:", configToSend);
                console.log("Weight params:", weightParams.map(p => `${p}=${configToSend[p]}`).join(', '));
                const response = await fetch('/run', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify(configToSend)
                });
                
                console.log("Response status:", response.status);
                if (!response.ok) {
                    const errorText = await response.text();
                    console.error("Response error:", errorText);
                    status.innerText = "Error: " + response.status + " - " + errorText.substring(0, 100);
                    return;
                }
                
                const result = await response.json();
                console.log("Received result:", result);
                
                if (result.error) {
                    status.innerText = "Error: " + result.error;
                    console.error("Server error:", result.error);
                    return;
                }
                
                // 检查数据是否有效
                if (!result.trajectory || result.trajectory.length === 0) {
                    status.innerText = "Warning: Empty trajectory";
                    console.warn("Empty trajectory received");
                    return;
                }
                
                console.log("Trajectory points:", result.trajectory.length);
                lastResult = result;
                plotTrajectory(result.trajectory, result.reference_line, result.heuristic_trajectory, result.path_corridor);
                status.innerText = "Done";
            } catch (e) {
                status.innerText = "Network Error: " + e.message;
                console.error("Exception:", e);
            }
        }

        function plotTrajectory(traj, refLine, heuristicTraj, pathCorridor) {
            console.log("plotTrajectory called with:", {
                traj: traj ? traj.length : 0,
                refLine: refLine ? refLine.length : 0,
                heuristicTraj: heuristicTraj ? heuristicTraj.length : 0,
                pathCorridor: pathCorridor ? pathCorridor.length : 0
            });
            
            if (!traj || traj.length === 0) {
                console.error("No trajectory data to plot!");
                return;
            }
            
            const data = [];

            // Plot Reference Line
            if (refLine && refLine.length > 0) {
                const refX = refLine.map(p => p.x);
                const refY = refLine.map(p => p.y);
                data.push({
                    x: refX,
                    y: refY,
                    mode: 'lines',
                    type: 'scatter',
                    name: 'Reference Line',
                    line: {color: 'gray', dash: 'dash'}
                });
            }

            // Plot Heuristic Trajectory
            if (heuristicTraj && heuristicTraj.length > 0) {
                const hX = heuristicTraj.map(p => p.x);
                const hY = heuristicTraj.map(p => p.y);
                data.push({
                    x: hX,
                    y: hY,
                    mode: 'lines+markers',
                    type: 'scatter',
                    name: 'Heuristic Trajectory',
                    line: {color: 'green'}
                });
            }

            // Plot Optimized Trajectory
            if (traj && traj.length > 0) {
                const x = traj.map(p => p.x);
                const y = traj.map(p => p.y);
                data.push({
                    x: x,
                    y: y,
                    mode: 'lines+markers',
                    type: 'scatter',
                    name: 'Optimized Trajectory',
                    line: {color: 'blue'}
                });
            }
            
            const layout = {
                title: 'Optimized Trajectory',
                xaxis: {title: 'X (m)', scaleanchor: "y", scaleratio: 1, range: [-50, 50]},
                yaxis: {title: 'Y (m)', range: [-50, 50]},
                margin: {t: 40, r: 20, b: 40, l: 40},
                dragmode: 'pan',
                scrollZoom: true
            };
            
            const config = {scrollZoom: true, responsive: true};
            Plotly.newPlot('plot', data, layout, config);

            // Plot Kappa
            const kappaData = [];
            if (traj && traj.length > 0) {
                const s = traj.map(p => p.s);
                const k = traj.map(p => p.kappa);
                kappaData.push({
                    x: s,
                    y: k,
                    mode: 'lines+markers',
                    type: 'scatter',
                    name: 'Kappa',
                    line: {color: 'red'}
                });
            }
            if (heuristicTraj && heuristicTraj.length > 0) {
                const s = heuristicTraj.map(p => p.s);
                const k = heuristicTraj.map(p => p.kappa);
                kappaData.push({
                    x: s,
                    y: k,
                    mode: 'lines+markers',
                    type: 'scatter',
                    name: 'Heuristic Kappa',
                    line: {color: 'orange'}
                });
            }
            
            const kappaLayout = {
                title: 'Curvature (Kappa)',
                xaxis: {title: 'S (m)'},
                yaxis: {title: 'Kappa (1/m)', range: [-0.4, 0.4]},
                margin: {t: 40, r: 20, b: 40, l: 40},
                dragmode: 'pan',
                scrollZoom: true
            };
            
            Plotly.newPlot('plot-kappa', kappaData, kappaLayout, config);

            // Plot Dkappa
            const dkappaData = [];
            if (traj && traj.length > 0) {
                const s = traj.map(p => p.s);
                const dk = traj.map(p => p.dkappa);
                dkappaData.push({
                    x: s,
                    y: dk,
                    mode: 'lines+markers',
                    type: 'scatter',
                    name: 'Dkappa',
                    line: {color: 'green'}
                });
            }
            if (heuristicTraj && heuristicTraj.length > 0) {
                const s = heuristicTraj.map(p => p.s);
                const dk = heuristicTraj.map(p => p.dkappa);
                dkappaData.push({
                    x: s,
                    y: dk,
                    mode: 'lines+markers',
                    type: 'scatter',
                    name: 'Heuristic Dkappa',
                    line: {color: 'lightgreen'}
                });
            }
            
            const dkappaLayout = {
                title: 'Curvature Rate (Dkappa)',
                xaxis: {title: 'S (m)'},
                yaxis: {title: 'Dkappa (1/m^2)'},
                margin: {t: 40, r: 20, b: 40, l: 40},
                dragmode: 'pan',
                scrollZoom: true
            };
            
            Plotly.newPlot('plot-dkappa', dkappaData, dkappaLayout, config);

            // Plot Jerk
            const jerkData = [];
            if (traj && traj.length > 0) {
                const t = traj.map(p => p.t);
                const j = traj.map(p => p.jerk);
                jerkData.push({
                    x: t,
                    y: j,
                    mode: 'lines+markers',
                    type: 'scatter',
                    name: 'Jerk',
                    line: {color: 'orange'}
                });
            }
            
            const jerkLayout = {
                title: 'Jerk',
                xaxis: {title: 'Time (s)'},
                yaxis: {title: 'Jerk (m/s^3)'},
                margin: {t: 40, r: 20, b: 40, l: 40},
                dragmode: 'pan',
                scrollZoom: true
            };
            
            Plotly.newPlot('plot-jerk', jerkData, jerkLayout, config);

            // Plot Velocity
            const vData = [];
            if (traj && traj.length > 0) {
                const t = traj.map(p => p.t);
                const v = traj.map(p => p.v);
                vData.push({
                    x: t,
                    y: v,
                    mode: 'lines+markers',
                    type: 'scatter',
                    name: 'Velocity',
                    line: {color: 'purple'}
                });
            }
            if (heuristicTraj && heuristicTraj.length > 0) {
                const t = heuristicTraj.map(p => p.t); 
                const v = heuristicTraj.map(p => p.v); 
                
                vData.push({
                    x: t,
                    y: v,
                    mode: 'lines+markers',
                    type: 'scatter',
                    name: 'Heuristic Velocity',
                    line: {color: 'violet'}
                });
            }

            const vLayout = {
                title: 'Velocity',
                xaxis: {title: 'Time (s)'},
                yaxis: {title: 'Velocity (m/s)'},
                margin: {t: 40, r: 20, b: 40, l: 40},
                dragmode: 'pan',
                scrollZoom: true
            };
            Plotly.newPlot('plot-v', vData, vLayout, config);

            // Plot Acceleration
            const aData = [];
            if (traj && traj.length > 0) {
                const t = traj.map(p => p.t);
                const a = traj.map(p => p.a);
                aData.push({
                    x: t,
                    y: a,
                    mode: 'lines+markers',
                    type: 'scatter',
                    name: 'Acceleration',
                    line: {color: 'brown'}
                });
            }
            const aLayout = {
                title: 'Acceleration',
                xaxis: {title: 'Time (s)'},
                yaxis: {title: 'Acceleration (m/s^2)'},
                margin: {t: 40, r: 20, b: 40, l: 40},
                dragmode: 'pan',
                scrollZoom: true
            };
            Plotly.newPlot('plot-a', aData, aLayout, config);

            // Plot Lateral Acceleration
            const latAccData = [];
            if (traj && traj.length > 0) {
                const t = traj.map(p => p.t);
                const lat_acc = traj.map(p => p.v * p.v * p.kappa);
                latAccData.push({
                    x: t,
                    y: lat_acc,
                    mode: 'lines+markers',
                    type: 'scatter',
                    name: 'Lat Acc',
                    line: {color: 'teal'}
                });
            }
            if (heuristicTraj && heuristicTraj.length > 0) {
                const t = heuristicTraj.map(p => p.t);
                const lat_acc = heuristicTraj.map(p => p.v * p.v * p.kappa);
                latAccData.push({
                    x: t,
                    y: lat_acc,
                    mode: 'lines+markers',
                    type: 'scatter',
                    name: 'Heuristic Lat Acc',
                    line: {color: 'cyan'}
                });
            }
            
            const latAccLayout = {
                title: 'Lateral Acceleration',
                xaxis: {title: 'Time (s)'},
                yaxis: {title: 'Lat Acc (m/s^2)'},
                margin: {t: 40, r: 20, b: 40, l: 40},
                dragmode: 'pan',
                scrollZoom: true
            };
            Plotly.newPlot('plot-lat-acc', latAccData, latAccLayout, config);

            // Plot l
            const lData = [];
            if (traj && traj.length > 0) {
                const t = traj.map(p => p.t);
                const l = traj.map(p => p.l);
                lData.push({
                    x: t,
                    y: l,
                    mode: 'lines+markers',
                    type: 'scatter',
                    name: 'l',
                    line: {color: 'magenta'}
                });

                // Plot Corridor Bounds
                if (pathCorridor && pathCorridor.length > 0) {
                    const len = Math.min(t.length, pathCorridor.length);
                    const t_sliced = t.slice(0, len);
                    const lb = pathCorridor.slice(0, len).map(p => p.hard_lb);
                    const ub = pathCorridor.slice(0, len).map(p => p.hard_ub);

                    lData.push({
                        x: t_sliced,
                        y: lb,
                        mode: 'lines',
                        type: 'scatter',
                        name: 'Lower Bound',
                        line: {color: 'red', dash: 'dash'}
                    });
                    lData.push({
                        x: t_sliced,
                        y: ub,
                        mode: 'lines',
                        type: 'scatter',
                        name: 'Upper Bound',
                        line: {color: 'red', dash: 'dash'}
                    });
                }
            }
            const lLayout = {
                title: 'l',
                xaxis: {title: 'Time (s)'},
                yaxis: {title: 'l (m)'},
                margin: {t: 40, r: 20, b: 40, l: 40},
                dragmode: 'pan',
                scrollZoom: true
            };
            Plotly.newPlot('plot-l', lData, lLayout, config);

            // Plot Heading
            const headingData = [];
            if (traj && traj.length > 0) {
                const t = traj.map(p => p.t);
                const theta = traj.map(p => p.theta);
                headingData.push({
                    x: t,
                    y: theta,
                    mode: 'lines+markers',
                    type: 'scatter',
                    name: 'Heading',
                    line: {color: 'blue'}
                });
            }
            if (heuristicTraj && heuristicTraj.length > 0) {
                const t = heuristicTraj.map(p => p.t);
                const theta = heuristicTraj.map(p => p.theta);
                headingData.push({
                    x: t,
                    y: theta,
                    mode: 'lines+markers',
                    type: 'scatter',
                    name: 'Heuristic Heading',
                    line: {color: 'green'}
                });
            }
            
            const headingLayout = {
                title: 'Heading',
                xaxis: {title: 'Time (s)'},
                yaxis: {title: 'Heading (rad)'},
                margin: {t: 40, r: 20, b: 40, l: 40},
                dragmode: 'pan',
                scrollZoom: true
            };
            Plotly.newPlot('plot-heading', headingData, headingLayout, config);
        }

        // Initialize
        fetch('/config')
            .then(r => r.json())
            .then(config => {
                createSliders(config.cilqr_trajectory_conf, config);
                runSimulation();
            });

    </script>
</body>
</html>
"""

class RequestHandler(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
        parsed_path = urlparse(self.path)
        path = parsed_path.path
        print(f"Received GET request for: {self.path}", file=sys.stderr)

        if path == '/' or path == '/index.html':
            self.send_response(200)
            self.send_header('Content-type', 'text/html; charset=utf-8')
            self.end_headers()
            self.wfile.write(HTML_TEMPLATE.encode())
        elif path == '/config':
            self.send_response(200)
            self.send_header('Content-type', 'application/json; charset=utf-8')
            self.end_headers()
            
            config = {}
            if os.path.exists(CONFIG_PATH):
                with open(CONFIG_PATH, 'r') as f:
                    try:
                        config = json.load(f)
                    except:
                        pass
            
            # Reset to defaults
            for k, v in DEFAULT_VALUES.items():
                if k == "cilqr_trajectory_conf":
                    if k not in config: config[k] = {}
                    for sub_k, sub_v in v.items():
                        config[k][sub_k] = sub_v
                else:
                    config[k] = v
            
            # Write back to file
            with open(CONFIG_PATH, 'w') as f:
                json.dump(config, f, indent=4)

            self.wfile.write(json.dumps(config).encode())
        else:
            self.send_error(404)

    def do_POST(self):
        if self.path == '/run':
            content_length = int(self.headers['Content-Length'])
            post_data = self.rfile.read(content_length)
            new_params = json.loads(post_data)
            
            # 1. Read existing config
            if not os.path.exists(CONFIG_PATH):
                self.send_error(500, "Config file not found")
                return

            with open(CONFIG_PATH, 'r') as f:
                config = json.load(f)
            
            # 2. Update params
            print(f"Received params: {new_params}", file=sys.stderr)
            for k, v in new_params.items():
                if k in ['init_x', 'init_y', 'init_theta', 'init_kappa', 'init_v', 'init_a']:
                    config[k] = v
                    print(f"Updated {k} = {v}", file=sys.stderr)
                elif k == 'ref_mode':
                    config[k] = v
                    print(f"Updated {k} = {v}", file=sys.stderr)
                elif k in config['cilqr_trajectory_conf']:
                    config['cilqr_trajectory_conf'][k] = v
                    print(f"Updated cilqr_trajectory_conf.{k} = {v}", file=sys.stderr)
                else:
                    print(f"Warning: Unknown parameter {k} = {v}", file=sys.stderr)
            
            # 3. Write config back
            with open(CONFIG_PATH, 'w') as f:
                json.dump(config, f, indent=4)
            
            # 4. Run executable
            try:
                # Set up environment variables
                env = os.environ.copy()
                workspace_dir = os.path.dirname(BASE_DIR)
                lib_path = os.path.join(workspace_dir, "x86_64", "opt", "lib")
                gears_lib_path = os.path.join(workspace_dir, "gears", "x86_64", "lib")
                
                # Add build directories to LD_LIBRARY_PATH
                build_src_path = os.path.join(BASE_DIR, "build_x86_64_Release", "src")
                build_proto_path = os.path.join(BASE_DIR, "build_x86_64_Release", "proto")
                build_vp_path = os.path.join(BASE_DIR, "build_x86_64_Release", "vehicle_pose")
                
                current_ld_path = env.get("LD_LIBRARY_PATH", "")
                env["LD_LIBRARY_PATH"] = f"{build_src_path}:{build_proto_path}:{build_vp_path}:{lib_path}:{gears_lib_path}:{current_ld_path}"
                
                print(f"Running executable: {EXECUTABLE_PATH}", file=sys.stderr)
                
                result = subprocess.run(
                    [EXECUTABLE_PATH, CONFIG_PATH], 
                    capture_output=True, 
                    text=True, 
                    env=env
                )
                
                if result.returncode != 0:
                    print(f"Executable failed with code {result.returncode}", file=sys.stderr)
                    print(f"Stderr: {result.stderr}", file=sys.stderr)
                    self.send_response(500)
                    self.end_headers()
                    self.wfile.write(json.dumps({"error": f"Executable failed: {result.stderr}"}).encode())
                    return

                output = result.stdout
                # print(f"Output: {output}", file=sys.stderr) # Debug output
                
                # 5. Parse output
                start_marker = "JSON_START"
                end_marker = "JSON_END"
                start_index = output.find(start_marker)
                end_index = output.find(end_marker)
                
                if start_index != -1 and end_index != -1:
                    json_str = output[start_index + len(start_marker):end_index].strip()
                    self.send_response(200)
                    self.send_header('Content-type', 'application/json; charset=utf-8')
                    self.end_headers()
                    self.wfile.write(json_str.encode())
                else:
                    print("JSON markers not found in output", file=sys.stderr)
                    print(f"Full output: {output}", file=sys.stderr)
                    self.send_response(500)
                    self.end_headers()
                    self.wfile.write(json.dumps({"error": "Invalid output from executable (markers not found)"}).encode())
                    
            except Exception as e:
                print(f"Exception in do_POST: {e}", file=sys.stderr)
                self.send_response(500)
                self.end_headers()
                self.wfile.write(json.dumps({"error": str(e)}).encode())

print(f"Starting server at http://localhost:{PORT}")
socketserver.TCPServer.allow_reuse_address = True

try:
    with socketserver.TCPServer(("", PORT), RequestHandler) as httpd:
        print(f"Server started successfully!")
        print(f"Open your browser and navigate to: http://localhost:{PORT}")
        print(f"Press Ctrl+C to stop the server")
        httpd.serve_forever()
except OSError as e:
    if e.errno == 48 or "Address already in use" in str(e):
        print(f"\n{'='*60}", file=sys.stderr)
        print(f"ERROR: Port {PORT} is already in use!", file=sys.stderr)
        print(f"{'='*60}", file=sys.stderr)
        print(f"\nPossible solutions:", file=sys.stderr)
        print(f"1. Find and kill the process using port {PORT}:", file=sys.stderr)
        print(f"   lsof -ti:{PORT} | xargs kill -9", file=sys.stderr)
        print(f"   or on Linux:", file=sys.stderr)
        print(f"   sudo netstat -tlnp | grep {PORT}", file=sys.stderr)
        print(f"   sudo kill -9 <PID>", file=sys.stderr)
        print(f"\n2. Use a different port by modifying PORT in this script", file=sys.stderr)
        print(f"   (currently PORT = {PORT})", file=sys.stderr)
        print(f"\n3. Check if another instance of this server is running", file=sys.stderr)
        print(f"{'='*60}\n", file=sys.stderr)
    else:
        print(f"\n{'='*60}", file=sys.stderr)
        print(f"ERROR: Failed to start server on port {PORT}", file=sys.stderr)
        print(f"Error: {e}", file=sys.stderr)
        print(f"{'='*60}\n", file=sys.stderr)
    sys.exit(1)
except KeyboardInterrupt:
    print(f"\n\nServer stopped by user")
    sys.exit(0)
except Exception as e:
    print(f"\n{'='*60}", file=sys.stderr)
    print(f"ERROR: Unexpected error while starting server", file=sys.stderr)
    print(f"Error: {e}", file=sys.stderr)
    print(f"{'='*60}\n", file=sys.stderr)
    sys.exit(1)