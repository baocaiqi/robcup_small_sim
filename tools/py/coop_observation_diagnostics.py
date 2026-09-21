"""从本轮观测文件归纳阈值证据；只读比赛数据，不改变策略。"""
import collections
import csv
import gzip
import json
from pathlib import Path
import statistics
import sys

ROOT = Path(__file__).resolve().parents[2] / "docs/work/coop-observation-20260921"


def diagnose(stem):
    tasks = json.load(gzip.open(stem.with_suffix(".tasks.json.gz"), "rt", encoding="utf-8"))
    index = {(t["game"], t["team"], t["id"]): t for t in tasks}
    observed = collections.defaultdict(lambda: collections.defaultdict(set))
    examples = {}
    push_start, moderate_frames, moderate_receiving = {}, collections.defaultdict(set), collections.defaultdict(set)
    for row in csv.DictReader(gzip.open(stem.with_suffix(".csv.gz"), "rt", encoding="utf-8", newline="")):
        key = (int(row["game"]), row["team"], int(row["task"]))
        t = index[key]
        event = row["event"]
        if event == "followup" or row["result"] == "match_end":
            continue
        n = lambda k: float(row[k])
        frame = int(row["frame"])
        team = row["team"]
        def flag(name):
            observed[team][name].add(key)
            examples.setdefault(team + ":" + name, row)
        if int(row["reset_epoch"]) != int(t["events"]["created"]["reset_epoch"]):
            flag("cross_reset")
        if int(row["push"]):
            push_start.setdefault(key, frame)
            if frame > push_start[key] and int(row["carry_team"]) == (0 if team == "blue" else 1) and int(row["carry_id"]) == 1 and int(row["push_applied"]):
                flag("passer_push_after_observation")
        if event == "released":
            flag("release")
            if key not in observed[team]["passer_push_after_observation"]:
                flag("release_without_passer_impulse_evidence")
            if int(row["reset_epoch"]) != int(t["events"]["created"]["reset_epoch"]):
                flag("release_cross_reset")
        if event not in ("sample", "finished"):
            continue
        if event == "sample" and 0.3 <= n("threat") < 0.6 and (int(row["active"]) or int(row["control"])):
            moderate_frames[(key[0], team)].add(frame)
            flag("moderate_task")
            if row["phase"] == "1" and int(row["active"]):
                moderate_receiving[(key[0], team)].add(frame)
                flag("moderate_receiving_task")
        if int(row["active"]) and row["phase"] == "0" and int(row["push"]) and n("ahead") > 0:
            if n("progress") >= 6 and n("projection") >= 1 and 12 <= n("separation") < 14:
                flag("shadow_separation_12")
            if n("progress") >= 6 and 0.5 <= n("projection") < 1 and n("separation") >= 14:
                flag("shadow_speed_05")
            if 4 <= n("progress") < 6 and n("projection") >= 1 and n("separation") >= 14:
                flag("shadow_travel_4")
            if n("progress") >= 6 and n("projection") < 1 and n("physical_projection") >= 1 and n("separation") >= 14:
                flag("missed_release_physical_candidate")
        if int(row["active"]) and row["phase"] == "1" and n("receiver_distance") < 12:
            flag("receiving_near")
            if n("speed") > 3:
                flag("receiving_near_fast")
            if n("receiver_distance") >= n("opp_distance") or n("receiver_distance") >= n("mate_distance"):
                flag("receiving_near_not_nearest")
            if int(row["receive_frames"]) == 1:
                flag("receiving_one_evidence_frame")
        if event == "finished" and row["result"] == "success":
            flag("success")
            if key not in observed[team]["passer_push_after_observation"]:
                flag("success_without_passer_impulse_evidence")
    # 中威胁接应后80帧内丢球只表示相关，不表示反事实因果。
    goals = collections.Counter()
    previous_scores = collections.defaultdict(lambda: [0, 0])
    for line in stem.with_suffix(".log").read_text(encoding="utf-8").splitlines():
        if not line.startswith("SIM_EVENT "):
            continue
        d = {k: int(v) for k, v in (word.split("=") for word in line.split()[1:])}
        old = previous_scores[d["game"]]
        for team, new, prev in [("blue", d["score_yellow"], old[1]), ("yellow", d["score_blue"], old[0])]:
            if new > prev:
                goals[team + "_conceded"] += new - prev
                if any(d["frame"] - 80 <= f <= d["frame"] for f in moderate_frames[(d["game"], team)]):
                    goals[team + "_after_moderate_80"] += new - prev
                if any(d["frame"] - 80 <= f <= d["frame"] for f in moderate_receiving[(d["game"], team)]):
                    goals[team + "_after_moderate_receiving_80"] += new - prev
        previous_scores[d["game"]] = [d["score_blue"], d["score_yellow"]]
    summaries = {}
    for team in {t["team"] for t in tasks}:
        ts = [t for t in tasks if t["team"] == team]
        durations = sorted(t["duration"] for t in ts)
        counts = collections.Counter({k: len(v) for k, v in observed[team].items()})
        for name in ("shadow_separation_12", "shadow_speed_05", "shadow_travel_4"):
            counts[name + "_never_released"] = len(observed[team][name] - observed[team]["release"])
        counts.update({"tasks": len(ts), "ever_push_observation": sum(t["push_samples"] > 0 for t in ts),
            "preparing_finish": sum(t["events"]["finished"]["phase"] == "0" for t in ts),
            "wm_moving_samples": sum(t["wm_moving_samples"] for t in ts),
            "physical_moving_samples": sum(t["physical_moving_samples"] for t in ts),
            "moderate_frames": sum(len(v) for (g, side), v in moderate_frames.items() if side == team),
            "moderate_receiving_frames": sum(len(v) for (g, side), v in moderate_receiving.items() if side == team),
            "duration_median": statistics.median(durations), "duration_p90": durations[int(len(durations)*0.9)],
            "control_moving_passer_frames": sum(t["control_passer_moving"] for t in ts),
            "control_both_near_frames": sum(t["control_both_near_frames"] for t in ts),
            "followup_both_near_frames": sum(t["followup_both_near_frames"] for t in ts)})
        counts["flight_outcomes"] = dict(collections.Counter(t["events"]["finished"]["result"] for t in ts if "released" in t["events"]))
        counts["controls"] = [{"game": t["game"], "id": t["id"], "duration": t["control_duration"],
                               "exit": t["events"]["control_exited"]["result"],
                               "physical_push_samples": t["receiver_push_samples"],
                               "touch_samples": t["receiver_touch_samples"],
                               "control_start": t["events"]["control_entered"]}
                              for t in ts if "control_entered" in t["events"]]
        summaries[team] = counts
    result = {"summary": summaries, "goals": goals, "examples": examples}
    stem.with_suffix(".diagnostics.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    print(stem.name, json.dumps({"summary": {k: {a: b for a, b in v.items() if a != "controls"} for k, v in summaries.items()}, "goals": goals}, ensure_ascii=False))


if __name__ == "__main__":
    for name in sys.argv[1:]:
        diagnose(ROOT / name)
