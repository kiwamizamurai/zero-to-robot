// Franka Panda でブロックを箱に片付けるシミュレーション。実体は ../_web/panda.js
// (pick_place.py の移植。panda_rrt などの Panda 系アプリも同じ土台を使う)。
export { BLOCKS, BOX_CENTER, BOX_HALF, BOX_HEIGHT } from "../_web/panda.js";
import { PandaCell } from "../_web/panda.js";

export class PickPlaceSim extends PandaCell {}
export { PickPlaceSim as default };
