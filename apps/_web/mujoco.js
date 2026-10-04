// MuJoCo WASM の読み込みと、モデルファイル(XML・メッシュ)の仮想ファイルシステムへの配置。
// ブラウザでも Node(headless のテスト)でも同じコードで動く。
import loadMujoco from "./vendor/mujoco.js";

export async function initMujoco() {
  return loadMujoco();
}

// ブラウザ用: base(import.meta.url など)からの相対パスで fetch する
export const browserLoader = (base) => async (file) => {
  const url = new URL(file, base);
  const res = await fetch(url);
  if (!res.ok) throw new Error(`${file} を読み込めません (${res.status})`);
  return new Uint8Array(await res.arrayBuffer());
};

// Node 用: ディスクから読む。base は file:// URL か絶対パス
export const nodeLoader = (base) => async (file) => {
  const { readFile } = await import("node:fs/promises");
  const { fileURLToPath } = await import("node:url");
  const root = String(base).startsWith("file:") ? fileURLToPath(base) : base;
  const { join, dirname } = await import("node:path");
  const dir = root.endsWith("/") ? root : dirname(root);
  return new Uint8Array(await readFile(join(dir, file)));
};

// files: 仮想ファイルシステム上のパス(prefix からの相対)の配列。load(file) は Uint8Array を返す
export async function mountFiles(mujoco, prefix, files, load) {
  for (const f of files) {
    const path = `${prefix}/${f}`;
    const dir = path.split("/").slice(0, -1).join("/");
    mujoco.FS.mkdirTree(dir, 0o777);
    mujoco.FS.writeFile(path, await load(f));
  }
}

export function writeText(mujoco, path, text) {
  mujoco.FS.mkdirTree(path.split("/").slice(0, -1).join("/") || "/", 0o777);
  mujoco.FS.writeFile(path, text);
}

export const siteId = (mj, model, name) => mj.mj_name2id(model, mj.mjtObj.mjOBJ_SITE.value, name);
export const bodyId = (mj, model, name) => mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY.value, name);
export const geomId = (mj, model, name) => mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM.value, name);
export const jointId = (mj, model, name) => mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT.value, name);
