import { ComponentFixture, TestBed } from '@angular/core/testing';

import { BobInfoComponent } from './bob-info.component';

describe('BobInfoComponent', () => {
  let component: BobInfoComponent;
  let fixture: ComponentFixture<BobInfoComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [ BobInfoComponent ]
    })
    .compileComponents();

    fixture = TestBed.createComponent(BobInfoComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
