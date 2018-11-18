using System.Data.Entity;
using System.Linq;
using System.Net;
using System.Web.Mvc;
using VehicleRouting.Models;

namespace VehicleRouting.Controllers
{
    public class ProductsController : Controller
    {
        private VehicleDbContext db = new VehicleDbContext();

        // GET: Products
        public ActionResult Index()
        {
            return this.View(this.db.Products.ToList());
        }

        // GET: Products/Details/5
        public ActionResult Details(int? id)
        {
            if (id == null)
            {
                return new HttpStatusCodeResult(HttpStatusCode.BadRequest);
            }
            Product product = this.db.Products.Find(id);
            if (product == null)
            {
                return this.HttpNotFound();
            }
            return this.View(product);
        }

        // GET: Products/Create
        public ActionResult Create()
        {
            return this.View();
        }

        // POST: Products/Create
        // To protect from overposting attacks, please enable the specific properties you want to bind to, for 
        // more details see http://go.microsoft.com/fwlink/?LinkId=317598.
        [HttpPost]
        [ValidateAntiForgeryToken]
        public ActionResult Create([Bind(Include = "ID,Name,Weight")] Product product)
        {
            if (this.ModelState.IsValid)
            {
                this.db.Products.Add(product);
                this.db.SaveChanges();
                return this.RedirectToAction("Index");
            }

            return this.View(product);
        }

        // GET: Products/Edit/5
        public ActionResult Edit(int? id)
        {
            if (id == null)
            {
                return new HttpStatusCodeResult(HttpStatusCode.BadRequest);
            }
            Product product = this.db.Products.Find(id);
            if (product == null)
            {
                return this.HttpNotFound();
            }
            return this.View(product);
        }

        // POST: Products/Edit/5
        // To protect from overposting attacks, please enable the specific properties you want to bind to, for 
        // more details see http://go.microsoft.com/fwlink/?LinkId=317598.
        [HttpPost]
        [ValidateAntiForgeryToken]
        public ActionResult Edit([Bind(Include = "ID,Name,Weight")] Product product)
        {
            if (this.ModelState.IsValid)
            {
                this.db.Entry(product).State = EntityState.Modified;
                this.db.SaveChanges();
                return this.RedirectToAction("Index");
            }
            return this.View(product);
        }

        // GET: Products/Delete/5
        public ActionResult Delete(int? id)
        {
            if (id == null)
            {
                return new HttpStatusCodeResult(HttpStatusCode.BadRequest);
            }
            Product product = this.db.Products.Find(id);
            if (product == null)
            {
                return this.HttpNotFound();
            }
            return this.View(product);
        }

        // POST: Products/Delete/5
        [HttpPost, ActionName("Delete")]
        [ValidateAntiForgeryToken]
        public ActionResult DeleteConfirmed(int id)
        {
            Product product = this.db.Products.Find(id);
            this.db.Products.Remove(product);
            this.db.SaveChanges();
            return this.RedirectToAction("Index");
        }

        protected override void Dispose(bool disposing)
        {
            if (disposing)
            {
                this.db.Dispose();
            }
            base.Dispose(disposing);
        }
    }
}
