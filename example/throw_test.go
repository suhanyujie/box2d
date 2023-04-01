package example

import (
	"github.com/ByteArena/box2d"
	"testing"
)

// https://juejin.cn/post/6844903448371003406
func TestCollision1(t *testing.T) {
	// Define the gravity vector.
	gravity := box2d.MakeB2Vec2(0.0, -10.0)
	// Construct a world object, which will hold and simulate the rigid bodies.
	world := box2d.MakeB2World(gravity)
	bodyDef := box2d.NewB2BodyDef()
	bodyDef.Type = box2d.B2BodyType.B2_dynamicBody
	fixDef := box2d.NewB2Fixture()
	fixDef.SetDensity(1.0)
	fixDef.SetFriction(0.9)
	fixDef.SetRestitution(1.0)
	fixDef.M_shape = box2d.NewB2CircleShape()
	fixDef.M_shape.(*box2d.B2CircleShape).SetRadius(50 / PTM_RATIO)
	bodyDef.Position.Set(250/PTM_RATIO, 0/PTM_RATIO)
	body := world.CreateBody(bodyDef)
	body.CreateFixture(fixDef.M_shape, fixDef.M_density)
}
