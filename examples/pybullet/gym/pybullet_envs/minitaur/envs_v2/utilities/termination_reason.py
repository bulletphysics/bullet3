"""Enum for classifying the termination reason of an episode."""

import enum
import gin


@gin.constants_from_enum
class TerminationReason(enum.IntEnum):
  """Enum that identifies termination reasons of an episode.

  For any new termination reason added here, please update the corresponding
  termination reward files to make sure it is used properly.
  """
  UNKNOWN = 0
  STEP_LIMIT = 1
  WALL_COLLISION = 2
  BAD_LOCATION = 3
  AGENT_COLLISION = 4
  GOAL_REACHED = 5
  INVALID_STEP_REVERT_AND_CONTINUE = 6
  INVALID_EPISODE = 7
  RUN_TIME_LIMIT = 8
  NOT_ADVANCING_LIMIT = 9
  NOT_LOCALIZED = 10


COLORMAP = {
    TerminationReason.UNKNOWN: (64, 64, 64),  # Dark gray.
    TerminationReason.STEP_LIMIT: (128, 64, 192),  # Purple.
    TerminationReason.WALL_COLLISION: (255, 64, 128),  # Bright red.
    TerminationReason.BAD_LOCATION: (255, 0, 192),  # Magenta.
    TerminationReason.AGENT_COLLISION: (255, 128, 0),  # Orange.
    TerminationReason.GOAL_REACHED: (96, 255, 96),  # Bright green.
    TerminationReason.INVALID_STEP_REVERT_AND_CONTINUE: (255, 255,
                                                         255),  # White.
    TerminationReason.INVALID_EPISODE: (0, 0, 0),  # Black.
    TerminationReason.RUN_TIME_LIMIT: (128, 64, 192),  # Purple.
    TerminationReason.NOT_ADVANCING_LIMIT: (128, 64, 192),  # Purple.
    TerminationReason.NOT_LOCALIZED: (255, 0, 192),  # Magenta.
}