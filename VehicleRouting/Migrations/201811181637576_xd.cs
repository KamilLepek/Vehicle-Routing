namespace VehicleRouting.Migrations
{
    using System.Data.Entity.Migrations;
    
    public partial class xd : DbMigration
    {
        public override void Up()
        {
            this.CreateTable(
                "dbo.Products",
                c => new
                    {
                        ID = c.Int(nullable: false, identity: true),
                        Name = c.String(),
                        Weight = c.Int(nullable: false),
                    })
                .PrimaryKey(t => t.ID);
            
        }
        
        public override void Down()
        {
            this.DropTable("dbo.Products");
        }
    }
}
