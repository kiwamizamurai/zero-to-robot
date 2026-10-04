// MuJoCo のモデルを three.js で描く共通ビューア(z 上向き。座標変換はいらない)。
//
//   const v = createViewer(canvas, { camera: [1.45, 0.95, 1.05], target: [0.3, -0.05, 0.28] });
//   v.build(model);        // モデルの geom からメッシュを作る(作り直すたびに呼ぶ)
//   v.sync(data);          // 毎フレーム、物理の状態を反映
//   v.render();
// geom の色: rgba を明示したものはその色。既定色(0.5,0.5,0.5)のものは neutral(Panda の白い外装など)。
import * as THREE from "three";
import { OrbitControls } from "three/addons/OrbitControls.js";

const T = { PLANE: 0, HFIELD: 1, SPHERE: 2, CAPSULE: 3, ELLIPSOID: 4, CYLINDER: 5, BOX: 6, MESH: 7 };

function meshGeometry(model, id) {
  const v0 = model.mesh_vertadr[id], nv = model.mesh_vertnum[id];
  const f0 = model.mesh_faceadr[id], nf = model.mesh_facenum[id];
  const g = new THREE.BufferGeometry();
  g.setAttribute("position", new THREE.BufferAttribute(Float32Array.from(model.mesh_vert.subarray(3 * v0, 3 * (v0 + nv))), 3));
  g.setIndex(Array.from(model.mesh_face.subarray(3 * f0, 3 * (f0 + nf))));
  g.computeVertexNormals();
  return g;
}

// MuJoCo の円柱・カプセルは z 軸方向。three.js は y 軸方向なので x まわりに 90° 回しておく
function zAligned(g) { g.rotateX(Math.PI / 2); return g; }

export function geometryFor(model, i) {
  const type = model.geom_type[i];
  const s = model.geom_size.subarray(3 * i, 3 * i + 3);
  switch (type) {
    case T.PLANE: return new THREE.PlaneGeometry(6, 6);
    case T.SPHERE: return new THREE.SphereGeometry(s[0], 24, 16);
    case T.CAPSULE: return zAligned(new THREE.CapsuleGeometry(s[0], 2 * s[1], 8, 16));
    case T.ELLIPSOID: { const g = new THREE.SphereGeometry(1, 24, 16); g.scale(s[0], s[1], s[2]); return g; }
    case T.CYLINDER: return zAligned(new THREE.CylinderGeometry(s[0], s[0], 2 * s[1], 24));
    case T.BOX: return new THREE.BoxGeometry(2 * s[0], 2 * s[1], 2 * s[2]);
    case T.MESH: return meshGeometry(model, model.geom_dataid[i]);
    default: return null;
  }
}

export function createViewer(canvas, { camera: camPos = [1.45, 0.95, 1.05], target = [0.3, -0.05, 0.28], neutral = 0xeceff2, fov = 40 } = {}) {
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, preserveDrawingBuffer: true });
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
  renderer.shadowMap.enabled = true;
  const scene = new THREE.Scene();
  const dark = matchMedia("(prefers-color-scheme: dark)").matches;
  scene.background = new THREE.Color(dark ? 0x16191c : 0xe9e6de);
  const camera = new THREE.PerspectiveCamera(fov, 4 / 3, 0.02, 40);
  camera.up.set(0, 0, 1);
  camera.position.set(...camPos);
  const controls = new OrbitControls(camera, canvas);
  controls.target.set(...target);
  controls.enableDamping = true;
  scene.add(new THREE.HemisphereLight(0xffffff, 0x8a8a90, 1.1));
  const sun = new THREE.DirectionalLight(0xffffff, 1.6);
  sun.position.set(0.8, -0.6, 2);
  sun.castShadow = true;
  sun.shadow.mapSize.set(2048, 2048);
  Object.assign(sun.shadow.camera, { left: -1.5, right: 1.5, top: 1.5, bottom: -1.5, near: 0.1, far: 6 });
  scene.add(sun);

  let items = [], extras = [];
  const mat4 = new THREE.Matrix4();

  const api = {
    THREE, renderer, scene, camera, controls,
    // 物理の geom をメッシュにする。hide(i, name) が true を返す geom は描かない
    build(model, { hide = null, colorOf = null } = {}) {
      for (const it of items) { scene.remove(it.mesh); it.mesh.geometry.dispose(); it.mesh.material.dispose(); }
      items = [];
      for (let i = 0; i < model.ngeom; i++) {
        if (hide?.(i)) continue;
        const geometry = geometryFor(model, i);
        if (!geometry) continue;
        const rgba = Array.from(model.geom_rgba.subarray(4 * i, 4 * i + 4));
        const explicit = Math.abs(rgba[0] - 0.5) > 1e-6 || Math.abs(rgba[1] - 0.5) > 1e-6;
        const color = colorOf?.(i, explicit, rgba) ?? (explicit ? new THREE.Color(rgba[0], rgba[1], rgba[2]) : new THREE.Color(neutral));
        const material = new THREE.MeshStandardMaterial({ color, roughness: 0.6, metalness: 0.05, transparent: rgba[3] < 1, opacity: rgba[3] });
        const mesh = new THREE.Mesh(geometry, material);
        mesh.matrixAutoUpdate = false;
        mesh.castShadow = model.geom_type[i] !== T.PLANE;
        mesh.receiveShadow = true;
        scene.add(mesh);
        items.push({ mesh, i });
      }
    },
    sync(data) {
      const p = data.geom_xpos, R = data.geom_xmat;
      for (const { mesh, i } of items) {
        mat4.set(R[9 * i], R[9 * i + 1], R[9 * i + 2], p[3 * i],
          R[9 * i + 3], R[9 * i + 4], R[9 * i + 5], p[3 * i + 1],
          R[9 * i + 6], R[9 * i + 7], R[9 * i + 8], p[3 * i + 2], 0, 0, 0, 1);
        mesh.matrix.copy(mat4);
      }
    },
    // 物理に無い飾り(軌跡の線、目印の球など)を足す。clearExtras でまとめて消す
    addExtra(obj) { scene.add(obj); extras.push(obj); return obj; },
    clearExtras() {
      for (const o of extras) { scene.remove(o); o.geometry?.dispose(); o.material?.dispose(); }
      extras = [];
    },
    line(points, color = 0xffaa00, width = 1) {
      const g = new THREE.BufferGeometry().setFromPoints(points.map((p) => new THREE.Vector3(...p)));
      return api.addExtra(new THREE.Line(g, new THREE.LineBasicMaterial({ color, linewidth: width })));
    },
    sphere(pos, r = 0.01, color = 0xffaa00) {
      const m = new THREE.Mesh(new THREE.SphereGeometry(r, 16, 12), new THREE.MeshBasicMaterial({ color }));
      m.position.set(...pos);
      return api.addExtra(m);
    },
    resize() {
      const w = canvas.clientWidth, h = canvas.clientHeight;
      if (w && canvas.width !== Math.floor(w * renderer.getPixelRatio())) {
        renderer.setSize(w, h, false);
        camera.aspect = w / h;
        camera.updateProjectionMatrix();
      }
    },
    render() { api.resize(); controls.update(); renderer.render(scene, camera); },
    // 別視点のカメラ(天井カメラなど)で描いた画像を RGBA で返す。物理の画面は壊さない
    capture(pos, lookAt, { width = 320, height = 240, fov: f = 50 } = {}) {
      const cam = new THREE.PerspectiveCamera(f, width / height, 0.02, 40);
      cam.up.set(0, 0, 1);
      cam.position.set(...pos);
      cam.lookAt(...lookAt);
      const target = new THREE.WebGLRenderTarget(width, height);
      renderer.setRenderTarget(target);
      renderer.render(scene, cam);
      const buf = new Uint8Array(width * height * 4);
      renderer.readRenderTargetPixels(target, 0, 0, width, height, buf);
      renderer.setRenderTarget(null);
      target.dispose();
      return { width, height, data: buf, flipY: true }; // 行は下から上の順
    },
  };
  return api;
}
