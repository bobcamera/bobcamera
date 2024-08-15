import { ComponentFixture, TestBed } from '@angular/core/testing';

import { StreamDisplayComponent } from './stream-display.component';

describe('StreamDisplayComponent', () => {
  let component: StreamDisplayComponent;
  let fixture: ComponentFixture<StreamDisplayComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [ StreamDisplayComponent ]
    })
    .compileComponents();

    fixture = TestBed.createComponent(StreamDisplayComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
