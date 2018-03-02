package org.team3128.common.util.enums;

/**
 * Direction enum used for turns.
 * @author Jamie
 *
 */
public enum Direction
{
	RIGHT,
	LEFT;
	
	public Direction opposite() {
		return (this == Direction.RIGHT) ? Direction.LEFT : Direction.RIGHT;
	}
}
