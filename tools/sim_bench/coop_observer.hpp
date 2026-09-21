// 仅由 sim_bench.cpp 在 SimState 定义后包含；统计不得反馈给策略。
static FILE *g_coop = nullptr;
static bool g_correct_ball_history = false;
static int g_coop_game = 0, g_coop_frame = 0;
static uint64_t g_coop_seed = 0;
static const SimState *g_coop_state = nullptr;
static int g_coop_tail[2] = {};
static unsigned long g_coop_tail_task[2] = {};

static void coop_csv_header() {
    fprintf(g_coop, "game,seed,frame,team,task,event,result,phase,active,control,receiver,left,push,receive_frames,loose_frames,");
    fprintf(g_coop, "bx,by,vx,vy,physical_vx,physical_vy,px,py,rx,ry,tx,ty,progress,projection,separation,ahead,receiver_distance,speed,threat,we,whos,opp_distance,mate_distance,passer_vl,passer_vr,receiver_vl,receiver_vr,reset_epoch,carry_team,carry_id,touch_blue,touch_yellow,score_blue,score_yellow,physical_projection,push_applied\n");
}
static void coop_csv_row(const WorldModel &wm, const char *event, CoopOutcome result) {
    if (!g_coop || !g_coop_state) return;
    if (strcmp(event, "control_exited") == 0 && result != CoopOutcome::MatchEnd) {
        int team = wm.ctx.is_blue ? 0 : 1;
        g_coop_tail[team] = 20; g_coop_tail_task[team] = wm.coop_stats.created;
    }
    const auto &t = wm.coop_pass_task;
    int id = t.receiver_id;
    if (id < 2 || id > 4) return;
    const auto &p = wm.home[1]; const auto &r = wm.home[id];
    const auto &s = *g_coop_state;
    double od = 1e9, md = 1e9;
    for (int i = 0; i < 5; ++i) {
        od = std::min(od, std::hypot(wm.opp[i].x - wm.ball.x, wm.opp[i].y - wm.ball.y));
        if (i != id) md = std::min(md, std::hypot(wm.home[i].x - wm.ball.x, wm.home[i].y - wm.ball.y));
    }
    double progress = (wm.ball.x - t.push_ball_x) * t.push_dir_x + (wm.ball.y - t.push_ball_y) * t.push_dir_y;
    double projection = wm.ball.vx * t.push_dir_x + wm.ball.vy * t.push_dir_y;
    double ahead = (wm.ball.x - p.x) * t.push_dir_x + (wm.ball.y - p.y) * t.push_dir_y;
    fprintf(g_coop, "%d,%llu,%d,%s,%lu,%s,%s,%d,%d,%d,%d,%d,%d,%d,%d,", g_coop_game,
        (unsigned long long)g_coop_seed, g_coop_frame, wm.ctx.is_blue ? "blue" : "yellow", wm.coop_stats.created,
        event, result == CoopOutcome::Count ? "none" : coop_outcome_name(result), (int)t.phase,
        (int)t.active, (int)wm.coop_ball_control.active, id, t.frames_left, (int)t.observing_push,
        t.receive_frames, wm.coop_ball_control.loose_frames);
    fprintf(g_coop, "%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,",
        wm.ball.x, wm.ball.y, wm.ball.vx, wm.ball.vy, s.bvx * kDt, s.bvy * kDt,
        p.x, p.y, r.x, r.y, t.rx, t.ry);
    fprintf(g_coop, "%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%d,%ld,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,",
        progress, projection, std::hypot(wm.ball.x-p.x, wm.ball.y-p.y), ahead,
        std::hypot(wm.ball.x-r.x, wm.ball.y-r.y), std::hypot(wm.ball.vx, wm.ball.vy),
        wm.threat_level, (int)wm.we_have_ball, wm.whos_ball, od, md, p.vl, p.vr, r.vl, r.vr);
    fprintf(g_coop, "%d,%d,%d,%u,%u,%d,%d,%.9g,%d\n", s.reset_epoch, s.carry_team, s.carry_id,
        s.touch_blue, s.touch_yellow, s.score_blue, s.score_yellow,
        (s.bvx * t.push_dir_x + s.bvy * t.push_dir_y) * kDt, (int)s.push_applied);
}
static void coop_sample(const WorldModel &wm) {
    if (wm.coop_pass_task.active || wm.coop_ball_control.active) coop_csv_row(wm, "sample", CoopOutcome::Count);
    int team = wm.ctx.is_blue ? 0 : 1;
    if (g_coop_tail[team] > 0) {
        if (wm.coop_stats.created == g_coop_tail_task[team]) coop_csv_row(wm, "followup", CoopOutcome::Count);
        --g_coop_tail[team];
    }
}
static void coop_print_result(WorldModel &wm) {
    // 比赛截止属于未完成样本；只在比赛结束后结账，不影响任一比赛帧。
    wm.coop_finish(CoopOutcome::MatchEnd);
    wm.coop_control_end(CoopOutcome::MatchEnd);
    const auto &c = wm.coop_stats;
    printf("COOP game=%d seed=%llu team=%s created=%lu released=%lu received=%lu control=%lu",
        g_coop_game, (unsigned long long)g_coop_seed, wm.ctx.is_blue ? "blue" : "yellow",
        c.created, c.released, c.received, c.control_entered);
    for (int i = 0; i < (int)CoopOutcome::Count; ++i)
        printf(" %s=%lu", coop_outcome_name((CoopOutcome)i), c.outcomes[i]);
    for (int i = 0; i < (int)CoopOutcome::Count; ++i)
        printf(" control_%s=%lu", coop_outcome_name((CoopOutcome)i), c.control_exits[i]);
    printf("\n");
}
