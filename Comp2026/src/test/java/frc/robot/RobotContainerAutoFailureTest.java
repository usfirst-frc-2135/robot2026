package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

import java.util.Collections;
import java.util.List;

import org.junit.jupiter.api.Test;

import com.pathplanner.lib.path.PathPlannerPath;

class RobotContainerAutoFailureTest
{
  @Test
  void evaluateAutoMapFallbackReasonReturnsMissingWhenAutoNameNull( )
  {
    assertEquals("AutoMapMissing", RobotContainer.evaluateAutoMapFallbackReason(null));
  }

  @Test
  void evaluateAutoMapFallbackReasonReturnsNullWhenAutoNamePresent( )
  {
    assertNull(RobotContainer.evaluateAutoMapFallbackReason("Start1_Stop"));
  }

  @Test
  void evaluateLoadedPathFallbackReasonReturnsEmptyWhenPathListNull( )
  {
    assertEquals("AutoPathListEmpty", RobotContainer.evaluateLoadedPathFallbackReason(null));
  }

  @Test
  void evaluateLoadedPathFallbackReasonReturnsEmptyWhenPathListEmpty( )
  {
    assertEquals("AutoPathListEmpty", RobotContainer.evaluateLoadedPathFallbackReason(Collections.emptyList( )));
  }

  @Test
  void evaluateLoadedPathFallbackReasonReturnsNullWhenPathListPresent( )
  {
    @SuppressWarnings("unchecked")
    List<PathPlannerPath> pathList = (List<PathPlannerPath>) (List<?>) List.of(new Object( ));
    assertNull(RobotContainer.evaluateLoadedPathFallbackReason(pathList));
  }

  @Test
  void evaluateBLineBuildFallbackReasonReturnsErrorWhenExceptionProvided( )
  {
    assertEquals("BLineBuildError", RobotContainer.evaluateBLineBuildFallbackReason(new RuntimeException("boom")));
  }

  @Test
  void evaluateBLineBuildFallbackReasonReturnsNullWhenNoError( )
  {
    assertNull(RobotContainer.evaluateBLineBuildFallbackReason(null));
  }
}
