// train.mjs が起こす作業者。お手本集め・学習・評価を並列に回す。
import { parentPort } from "node:worker_threads";
import { initMujoco, nodeLoader } from "../../_web/mujoco.js";
import { ImitationSim, NOISE_STD } from "../sim.js";
import { Policy } from "../policy.js";
import { trainMLP, toJSON } from "./mlp.mjs";
import { mulberry32 } from "../../_web/math.js";

const mujoco = await initMujoco();
const sim = await ImitationSim.create(mujoco, nodeLoader(new URL("../../_web/", import.meta.url)), { seed: 1 });

const tasks = {
  // placements: [[x, y, yaw, 揺れの seed], ...]。(状態, 指令) の組と、先生の成功数を返す
  collect({ placements, noisy }) {
    const X = [], Y = [];
    let ok = 0;
    for (const [x, y, yaw, seed] of placements) {
      sim.place(x, y, yaw);
      const rnd = mulberry32(seed);
      for (const _ of sim.runTeacher({ noiseStd: noisy ? NOISE_STD : 0, rnd, onSample: (o, c) => { X.push(...o); Y.push(...c); } }));
      ok += sim.success();
    }
    return { X: Float32Array.from(X), Y: Float32Array.from(Y), ok };
  },
  train({ X, Y, opts, meta }) {
    const net = trainMLP(X, Y, opts);
    return { json: toJSON(net, { ...meta, loss: net.loss, steps: net.steps, hidden: opts.hidden }), loss: net.loss };
  },
  // 学習済みの方策(JSON)で、各置き方を片付けさせる。teacher なら先生(手順書)で
  evaluate({ json, placements, teacher }) {
    const policy = teacher ? null : new Policy(json);
    return placements.map(([x, y, yaw]) => {
      sim.place(x, y, yaw);
      for (const _ of teacher ? sim.runTeacher() : sim.runPolicy(policy));
      return sim.success();
    });
  },
};

parentPort.on("message", ({ id, type, args }) => {
  try {
    parentPort.postMessage({ id, result: tasks[type](args) });
  } catch (e) {
    parentPort.postMessage({ id, error: String(e.stack ?? e) });
  }
});
parentPort.postMessage({ ready: true });
