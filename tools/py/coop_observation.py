"""运行固定种子完整比赛，核验任务记账，保存可复盘的观测产物。仅使用标准库。"""
import argparse
import collections
import csv
import gzip
import json
from pathlib import Path
import re
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "docs/work/coop-observation-20260921"


def analyze(stem):
    log = stem.with_suffix(".log").read_text(encoding="utf-8")
    matches, totals = [], {}
    for line in log.splitlines():
        if line.startswith("COOP "):
            d = dict(word.split("=", 1) for word in line.split()[1:])
            d = {k: v if k == "team" else int(v) for k, v in d.items()}
            outcomes = {k: v for k, v in d.items() if k not in
                        {"game", "seed", "team", "created", "released", "received", "control"}
                        and not k.startswith("control_")}
            assert sum(outcomes.values()) == d["created"], d
            assert d["received"] == d["success"] == d["control"] <= d["released"] <= d["created"], d
            assert sum(v for k, v in d.items() if k.startswith("control_")) == d["control"], d
            matches.append(d)
            team = totals.setdefault(d["team"], collections.Counter())
            team.update({k: v for k, v in d.items() if k not in {"game", "seed", "team"}})
    tasks = {}
    counts = collections.Counter()
    filename = stem.with_suffix(".csv")
    with filename.open(newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            assert None not in row, "CSV 列数不匹配"
            key = (int(row["game"]), row["team"], int(row["task"]))
            event = row["event"]
            task = tasks.setdefault(key, {"game": key[0], "team": key[1], "id": key[2],
                "seed": row["seed"], "events": {}, "samples": 0, "moderate_frames": 0,
                "near_receiver_frames": 0, "near_fast_frames": 0, "release_geometry_no_speed": 0,
                "physical_release_candidate": 0, "control_passer_moving": 0,
                "max_progress": 0, "max_separation": 0, "wm_moving_samples": 0,
                "physical_moving_samples": 0, "receiver_carry_samples": 0,
                "receiver_touch_samples": 0, "receiver_push_samples": 0,
                "push_samples": 0, "max_receive_frames": 0, "receiving_near_frames": 0,
                "receiving_near_fast_frames": 0, "control_both_near_frames": 0,
                "followup_both_near_frames": 0, "followup_samples": 0,
                "reset_epochs": [], "phase_frames": {}})
            get = lambda k: float(row[k])
            if event == "followup":
                task["followup_samples"] += 1
                task["followup_both_near_frames"] += get("separation") < 12 and get("receiver_distance") < 12
                continue
            epoch = int(row["reset_epoch"])
            if epoch not in task["reset_epochs"]:
                task["reset_epochs"].append(epoch)
            if event != "sample":
                assert event not in task["events"], (key, event, "同任务重复事件")
                task["events"][event] = row
                counts[(key[0], key[1], event)] += 1
                continue
            task["samples"] += 1
            phase = "control" if int(row["control"]) else row["phase"]
            task["phase_frames"][phase] = task["phase_frames"].get(phase, 0) + 1
            task["push_samples"] += bool(int(row["push"]))
            task["max_receive_frames"] = max(task["max_receive_frames"], int(row["receive_frames"]))
            task["max_progress"] = max(task["max_progress"], get("progress"))
            task["max_separation"] = max(task["max_separation"], get("separation"))
            task["moderate_frames"] += 0.3 <= get("threat") < 0.6
            task["near_receiver_frames"] += get("receiver_distance") < 12
            task["near_fast_frames"] += get("receiver_distance") < 12 and get("speed") > 3
            task["receiving_near_frames"] += phase == "1" and get("receiver_distance") < 12
            task["receiving_near_fast_frames"] += phase == "1" and get("receiver_distance") < 12 and get("speed") > 3
            task["wm_moving_samples"] += get("speed") >= 1
            pv = (get("physical_vx")**2 + get("physical_vy")**2)**0.5
            task["physical_moving_samples"] += pv >= 1
            geom = int(row["push"]) and get("progress") >= 6 and get("separation") >= 14 and get("ahead") > 0
            missing = geom and get("projection") < 1 and phase == "0"
            task["release_geometry_no_speed"] += bool(missing)
            # 物理方向投影需由相邻任务观察起点推导；此处仅做移动候选，不能称真实出球。
            physical_forward = get("physical_projection") >= 1 if "physical_projection" in row else pv >= 1
            task["physical_release_candidate"] += bool(missing and physical_forward)
            if missing and physical_forward and "example_missed_release" not in task:
                task["example_missed_release"] = row
            task["control_passer_moving"] += phase == "control" and (abs(get("passer_vl")) > 1e-6 or abs(get("passer_vr")) > 1e-6)
            task["control_both_near_frames"] += phase == "control" and get("separation") < 12 and get("receiver_distance") < 12
            team_id = 0 if row["team"] == "blue" else 1
            task["receiver_carry_samples"] += int(row["carry_team"]) == team_id and int(row["carry_id"]) == int(row["receiver"])
            task["receiver_push_samples"] += int(row["carry_team"]) == team_id and int(row["carry_id"]) == int(row["receiver"]) and int(row.get("push_applied", 0))
            mask = int(row["touch_blue"] if team_id == 0 else row["touch_yellow"])
            task["receiver_touch_samples"] += bool(mask & (1 << int(row["receiver"])))
    for d in matches:
        for event, field in [("created", "created"), ("released", "released"), ("control_entered", "control"), ("finished", "created")]:
            assert counts[(d["game"], d["team"], event)] == d[field], (d, event)
    for task in tasks.values():
        assert "created" in task["events"] and "finished" in task["events"], task
        task["duration"] = int(task["events"]["finished"]["frame"]) - int(task["events"]["created"]["frame"])
        if "control_entered" in task["events"]:
            task["control_duration"] = int(task["events"]["control_exited"]["frame"]) - int(task["events"]["control_entered"]["frame"])
    result = {"totals": totals, "matches": matches,
              "fit": next(line for line in log.splitlines() if line.startswith("FIT ")),
              "discipline": next(line for line in log.splitlines() if line.startswith("=== 禁区纪律"))}
    stem.with_suffix(".json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    with gzip.open(stem.with_suffix(".tasks.json.gz"), "wt", encoding="utf-8") as f:
        json.dump(list(tasks.values()), f, ensure_ascii=False)
    with filename.open("rb") as source, gzip.open(str(filename) + ".gz", "wb") as dest:
        shutil.copyfileobj(source, dest)
    # 压缩完成后只删除本次生成且可从 gzip 完整恢复的临时 CSV。
    filename.unlink()
    print(stem.name, result["fit"], json.dumps(totals, ensure_ascii=False))


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--mode", choices=["scripted", "yellow", "self"], required=True)
    p.add_argument("--history", choices=["original", "corrected"], required=True)
    p.add_argument("--games", type=int, required=True)
    p.add_argument("--seed", type=int, required=True)
    p.add_argument("--compare-baseline", action="store_true")
    args = p.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    stem = OUT / f"{args.history}_{args.mode}_{args.games}_seed{args.seed}"
    common = ["--games", str(args.games), "--frames", "24000", "--seed", str(args.seed), "--opp", args.mode]
    command = [str(ROOT / "build/Release/sim_bench.exe"), *common, "--coop-csv", str(stem.with_suffix(".csv"))]
    if args.history == "corrected":
        command.append("--correct-ball-history")
    with stem.with_suffix(".log").open("wb") as f:
        subprocess.run(command, stdout=f, stderr=subprocess.STDOUT, check=True, cwd=ROOT)
    if args.compare_baseline:
        assert args.history == "original"
        before_path = stem.with_suffix(".baseline.log")
        with before_path.open("wb") as f:
            subprocess.run([str(ROOT / "build/coop_observation/sim_bench_before.exe"), *common], stdout=f, stderr=subprocess.STDOUT, check=True, cwd=ROOT)
        before = before_path.read_text(encoding="utf-8")
        after = stem.with_suffix(".log").read_text(encoding="utf-8")
        # 原生逐场行包括比分、控球、射门、球位、门区、争球和角区救球。
        rows = lambda s: [line for line in s.splitlines() if re.match(r"\s+场\d+:", line)]
        assert len(rows(before)) == args.games and rows(before) == rows(after), "观测改变了比赛指标"
        print("baseline_equal", args.mode, args.games)
    analyze(stem)


if __name__ == "__main__":
    main()
