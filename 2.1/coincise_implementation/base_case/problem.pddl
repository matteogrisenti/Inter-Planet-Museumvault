(define (problem single-robot-problem-scalable-optimized)
  (:domain single-robot-optimized)

  ;; -------------------------------------------------------------------------
  ;; OBJECT DEFINITIONS
  ;; -------------------------------------------------------------------------
  (:objects
    ;; --- FIXED OBJECTS ---
    curator                     - robot
    pod1 pod2                   - pod
    entrance maintenance-tunnel hall-a hall-b cryo-chamber anti-vibration-pods-room stasis-lab - location
    
    ;; Artifact Types
    martian-core martian-generic martian-civilization asteroid-generic venus-generic - artifact-type

    ;; --- BASE GAME ARTIFACTS (~13 ITEMS) ---
    mart-nord-core-drill mart-sud-core-drill mart-east-core-drill mart-west-core-drill - artifact
    mart-sand-sample mart-north-pole-ice-sample mart-mysterious-egg - artifact
    mart-laser-gun mart-pink-hat - artifact
    asteroid-MG04TN-ice-sample asteroid-AD29TV-rock-sample - artifact
    venus-sand-sample venus-rock-sample - artifact

    ;; --- [BATCH 1] EXPANSION: THE "BIOLOGIST'S CACHE" (+10 ITEMS) ---
    batch1-bio-01 batch1-bio-02 batch1-bio-03 batch1-bio-04 batch1-bio-05 - artifact
    batch1-fragile-01 batch1-fragile-02 batch1-fragile-03 batch1-fragile-04 batch1-fragile-05 - artifact

    ;; --- [BATCH 2] EXPANSION: THE "HEAVY CORES" (+10 ITEMS) ---
    batch2-core-01 batch2-core-02 batch2-core-03 batch2-core-04 batch2-core-05 - artifact
    batch2-rescue-01 batch2-rescue-02 batch2-rescue-03 batch2-rescue-04 batch2-rescue-05 - artifact

    ;; --- [BATCH 3] EXPANSION: THE "SEISMIC NIGHTMARE" (+10 ITEMS) ---
    batch3-heavy-01 batch3-heavy-02 batch3-heavy-03 batch3-heavy-04 batch3-heavy-05 - artifact
    batch3-tiny-01  batch3-tiny-02  batch3-tiny-03  batch3-tiny-04  batch3-tiny-05 - artifact
  )

  ;; -------------------------------------------------------------------------
  ;; INITIAL STATE
  ;; -------------------------------------------------------------------------
  (:init
    ;; --- FIXED WORLD STATE ---
    (pod-at pod1 anti-vibration-pods-room) 
    (pod-at pod2 anti-vibration-pods-room)
    
    ;; --- TOPOLOGY ---
    (connected entrance maintenance-tunnel) (connected maintenance-tunnel entrance)
    (connected maintenance-tunnel hall-a) (connected hall-a maintenance-tunnel)
    (connected maintenance-tunnel hall-b) (connected hall-b maintenance-tunnel)
    (connected maintenance-tunnel cryo-chamber) (connected cryo-chamber maintenance-tunnel)
    (connected maintenance-tunnel anti-vibration-pods-room) (connected anti-vibration-pods-room maintenance-tunnel)
    (connected maintenance-tunnel stasis-lab) (connected stasis-lab maintenance-tunnel)

    ;; --- ATMOSPHERE ---
    ;; Only listing pressurized rooms. Maintenance-tunnel is implicitly vacuum.
    (is-pressurized entrance) (is-pressurized hall-a)
    (is-pressurized hall-b) (is-pressurized cryo-chamber) 
    (is-pressurized anti-vibration-pods-room) (is-pressurized stasis-lab)

    ;; --- SEISMIC SAFETY ---
    ;; Only listing seismic rooms. Others are implicitly safe.
    (is-seismic hall-b)

    ;; --- ROOM TYPES ---
    (is-standard-room hall-a) (is-standard-room hall-b) (is-standard-room entrance)
    (is-standard-room anti-vibration-pods-room) (is-standard-room maintenance-tunnel) (is-standard-room stasis-lab)
    (is-chill-room cryo-chamber)

    ;; --- ROBOT ---
    ;; Robot starts unsealed (implicit).
    (robot-at curator entrance)

    ;; -----------------------------------------------------------------------
    ;; BASE GAME ARTIFACT INIT
    ;; -----------------------------------------------------------------------
    (is-type mart-nord-core-drill martian-core) (is-type mart-sud-core-drill martian-core)
    (is-type mart-east-core-drill martian-core) (is-type mart-west-core-drill martian-core)
    (is-type mart-sand-sample martian-generic) (is-type mart-north-pole-ice-sample martian-generic)
    (is-type mart-mysterious-egg martian-generic) (is-type mart-laser-gun martian-civilization)
    (is-type mart-pink-hat martian-civilization) (is-type asteroid-MG04TN-ice-sample asteroid-generic)
    (is-type asteroid-AD29TV-rock-sample asteroid-generic) (is-type venus-sand-sample venus-generic)
    (is-type venus-rock-sample venus-generic)

    (artifact-at mart-nord-core-drill hall-a) (artifact-at mart-sud-core-drill hall-a)
    (artifact-at mart-east-core-drill hall-a) (artifact-at mart-west-core-drill hall-a)
    (artifact-at mart-north-pole-ice-sample hall-a) (artifact-at mart-mysterious-egg hall-a)
    (artifact-at asteroid-MG04TN-ice-sample hall-a)
    (artifact-at mart-sand-sample hall-b)
    (artifact-at mart-laser-gun hall-b)
    (artifact-at mart-pink-hat hall-b) (artifact-at asteroid-AD29TV-rock-sample hall-b)
    (artifact-at venus-sand-sample hall-b) (artifact-at venus-rock-sample hall-b)

    ;; Fragility
    (fragile mart-sand-sample) (fragile mart-laser-gun) (fragile mart-pink-hat)
    (fragile asteroid-AD29TV-rock-sample) (fragile venus-sand-sample) (fragile venus-rock-sample)

    ;; -----------------------------------------------------------------------
    ;; [BATCH 1] INIT
    ;; -----------------------------------------------------------------------
    (is-type batch1-bio-01 martian-generic) (is-type batch1-bio-02 martian-generic)
    (is-type batch1-bio-03 martian-generic) (is-type batch1-bio-04 martian-generic)
    (is-type batch1-bio-05 martian-generic)
    (is-type batch1-fragile-01 venus-generic) (is-type batch1-fragile-02 venus-generic)
    (is-type batch1-fragile-03 venus-generic) (is-type batch1-fragile-04 venus-generic)
    (is-type batch1-fragile-05 venus-generic)

    (artifact-at batch1-bio-01 hall-a) (artifact-at batch1-bio-02 hall-a)
    (artifact-at batch1-bio-03 hall-a) (artifact-at batch1-bio-04 hall-a)
    (artifact-at batch1-bio-05 hall-a)
    (artifact-at batch1-fragile-01 hall-b) (artifact-at batch1-fragile-02 hall-b)
    (artifact-at batch1-fragile-03 hall-b) (artifact-at batch1-fragile-04 hall-b)
    (artifact-at batch1-fragile-05 hall-b)

    (fragile batch1-fragile-01) (fragile batch1-fragile-02) (fragile batch1-fragile-03) 
    (fragile batch1-fragile-04) (fragile batch1-fragile-05)

    ;; -----------------------------------------------------------------------
    ;; [BATCH 2] INIT
    ;; -----------------------------------------------------------------------
    (is-type batch2-core-01 martian-core) (is-type batch2-core-02 martian-core)
    (is-type batch2-core-03 martian-core) (is-type batch2-core-04 martian-core)
    (is-type batch2-core-05 martian-core)
    (is-type batch2-rescue-01 asteroid-generic) (is-type batch2-rescue-02 asteroid-generic)
    (is-type batch2-rescue-03 asteroid-generic) (is-type batch2-rescue-04 asteroid-generic)
    (is-type batch2-rescue-05 asteroid-generic)

    (artifact-at batch2-core-01 hall-a) (artifact-at batch2-core-02 hall-a)
    (artifact-at batch2-core-03 hall-a) (artifact-at batch2-core-04 hall-a)
    (artifact-at batch2-core-05 hall-a)
    (artifact-at batch2-rescue-01 hall-b) (artifact-at batch2-rescue-02 hall-b)
    (artifact-at batch2-rescue-03 hall-b) (artifact-at batch2-rescue-04 hall-b)
    (artifact-at batch2-rescue-05 hall-b)

    (fragile batch2-rescue-01) (fragile batch2-rescue-02) (fragile batch2-rescue-03) 
    (fragile batch2-rescue-04) (fragile batch2-rescue-05)

    ;; -----------------------------------------------------------------------
    ;; [BATCH 3] INIT
    ;; -----------------------------------------------------------------------
    (is-type batch3-heavy-01 venus-generic) (is-type batch3-heavy-02 venus-generic)
    (is-type batch3-heavy-03 venus-generic) (is-type batch3-heavy-04 venus-generic)
    (is-type batch3-heavy-05 venus-generic)
    (is-type batch3-tiny-01 martian-generic) (is-type batch3-tiny-02 martian-generic)
    (is-type batch3-tiny-03 martian-generic) (is-type batch3-tiny-04 martian-generic)
    (is-type batch3-tiny-05 martian-generic)

    (artifact-at batch3-heavy-01 hall-b) (artifact-at batch3-heavy-02 hall-b)
    (artifact-at batch3-heavy-03 hall-b) (artifact-at batch3-heavy-04 hall-b)
    (artifact-at batch3-heavy-05 hall-b)
    (artifact-at batch3-tiny-01 hall-b) (artifact-at batch3-tiny-02 hall-b)
    (artifact-at batch3-tiny-03 hall-b) (artifact-at batch3-tiny-04 hall-b)
    (artifact-at batch3-tiny-05 hall-b)

    (fragile batch3-heavy-01) (fragile batch3-heavy-02) (fragile batch3-heavy-03) 
    (fragile batch3-heavy-04) (fragile batch3-heavy-05)
  )

  ;; -------------------------------------------------------------------------
  ;; GOAL STATE
  ;; -------------------------------------------------------------------------
  (:goal (and
    ;; --- BASE GAME GOALS ---
    (artifact-at mart-nord-core-drill stasis-lab) (cold mart-nord-core-drill)
    (artifact-at mart-sud-core-drill stasis-lab) (cold mart-sud-core-drill)
    (artifact-at mart-east-core-drill stasis-lab) (cold mart-east-core-drill)
    (artifact-at mart-west-core-drill stasis-lab) (cold mart-west-core-drill)

    (artifact-at mart-north-pole-ice-sample cryo-chamber)
    (artifact-at mart-mysterious-egg cryo-chamber)
    (artifact-at asteroid-MG04TN-ice-sample cryo-chamber)

    (artifact-at mart-sand-sample hall-a)
    (artifact-at mart-laser-gun hall-a)
    (artifact-at mart-pink-hat hall-a)
    (artifact-at asteroid-AD29TV-rock-sample hall-a)
    (artifact-at venus-sand-sample hall-a)
    (artifact-at venus-rock-sample hall-a)

    ;; -----------------------------------------------------------------------
    ;; [BATCH 1] GOALS
    ;; -----------------------------------------------------------------------
    (artifact-at batch1-bio-01 cryo-chamber) (artifact-at batch1-bio-02 cryo-chamber)
    (artifact-at batch1-bio-03 cryo-chamber) (artifact-at batch1-bio-04 cryo-chamber)
    (artifact-at batch1-bio-05 cryo-chamber)
    (artifact-at batch1-fragile-01 hall-a) (artifact-at batch1-fragile-02 hall-a)
    (artifact-at batch1-fragile-03 hall-a) (artifact-at batch1-fragile-04 hall-a)
    (artifact-at batch1-fragile-05 hall-a)

    ;; -----------------------------------------------------------------------
    ;; [BATCH 2] GOALS
    ;; -----------------------------------------------------------------------
    (artifact-at batch2-core-01 stasis-lab) (cold batch2-core-01)
    (artifact-at batch2-core-02 stasis-lab) (cold batch2-core-02)
    (artifact-at batch2-core-03 stasis-lab) (cold batch2-core-03)
    (artifact-at batch2-core-04 stasis-lab) (cold batch2-core-04)
    (artifact-at batch2-core-05 stasis-lab) (cold batch2-core-05)
    (artifact-at batch2-rescue-01 hall-a) (artifact-at batch2-rescue-02 hall-a)
    (artifact-at batch2-rescue-03 hall-a) (artifact-at batch2-rescue-04 hall-a)
    (artifact-at batch2-rescue-05 hall-a)

    ;; -----------------------------------------------------------------------
    ;; [BATCH 3] GOALS
    ;; -----------------------------------------------------------------------
    (artifact-at batch3-heavy-01 hall-a) (artifact-at batch3-heavy-02 hall-a)
    (artifact-at batch3-heavy-03 hall-a) (artifact-at batch3-heavy-04 hall-a)
    (artifact-at batch3-heavy-05 hall-a)
    (artifact-at batch3-tiny-01 hall-a) (artifact-at batch3-tiny-02 hall-a)
    (artifact-at batch3-tiny-03 hall-a) (artifact-at batch3-tiny-04 hall-a)
    (artifact-at batch3-tiny-05 hall-a)
  ))
)