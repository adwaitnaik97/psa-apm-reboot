@0x88491399d8a89f16;

struct PauseResumeCmd {
  pause @0 :Bool; # [0 - Currently paused, resume desired], [1 - Current resumed, pause desired]
}