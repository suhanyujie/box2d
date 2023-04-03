package example

import (
	"fmt"
	"testing"

	"github.com/ByteArena/box2d"
)

// https://juejin.cn/post/6844903448371003406
func GetWorld() *box2d.B2World {
	// Define the gravity vector.
	gravity := box2d.MakeB2Vec2(0.0, -10.0)
	// Construct a world object, which will hold and simulate the rigid bodies.
	world := box2d.MakeB2World(gravity)
	return &world
}

func SetBody1(world *box2d.B2World) *box2d.B2Body {
	bodyDef := box2d.NewB2BodyDef() // 刚体实例化
	bodyDef.Type = box2d.B2BodyType.B2_dynamicBody
	fixDef := box2d.NewB2Fixture()
	fixDef.SetDensity(1.0)
	fixDef.SetFriction(0.9)
	fixDef.SetRestitution(1.0)
	fixDef.M_shape = box2d.NewB2CircleShape()
	fixDef.M_shape.(*box2d.B2CircleShape).SetRadius(50 / PTM_RATIO)
	bodyDef.Position.Set(250/PTM_RATIO, 0/PTM_RATIO)
	body := world.CreateBody(bodyDef)
	body.CreateFixture(fixDef.M_shape, fixDef.M_density) // 为 body 套上材质

	return body
}

func SetBody2(world *box2d.B2World) *box2d.B2Body {
	bodyDef := box2d.NewB2BodyDef() // 刚体实例化
	bodyDef.Type = box2d.B2BodyType.B2_dynamicBody
	bodyDef.Position.Set(0., 4.)
	body := world.CreateBody(bodyDef)

	dynBox := box2d.NewB2PolygonShape()
	dynBox.SetAsBox(1., 1.)

	fixDef := box2d.NewB2Fixture()
	fixDef.SetDensity(1.0)
	fixDef.SetFriction(0.3)
	fixDef.SetRestitution(1.0)
	fixDef.M_shape = dynBox

	body.CreateFixture(fixDef.M_shape, fixDef.M_density) // 为 body 套上材质

	return body
}

func TestFrame1(t *testing.T) {
	var timeStep float64 = 1. / 60.
	world := GetWorld()
	body1 := SetBody2(world)
	// last := time.Now().UnixMilli()
	var animate func()
	animate = func() {
		// nowTime := time.Now()
		// var timeStep = nowTime.UnixMilli() - last
		// last = nowTime.UnixMilli()
		// velocityIterations 是对速率的纠正程度, 越高计算量越大.
		var velocityIterations = 6
		// positionIterations 是对位置的纠正程度, 越高计算量越大.
		var positionIterations = 2
		world.Step(timeStep, velocityIterations, positionIterations)
		world.ClearForces()
	}
	for i := 0; i < 60; i++ {
		animate()

		posi := body1.GetPosition()
		angle := body1.GetAngle()
		fmt.Printf("res posi x: %v, y:%v, angle:%v \n", posi.X, posi.Y, angle)
	}
}

func NewDynBody1() {
	//bodyDef := box2d.MakeB2BodyDef()
	//bodyDef.Type = box2d.MakeB2DynamicTree()
}
